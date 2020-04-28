use heapless::{
    consts::*,
    String,
    Vec
};

use core::fmt::Write;


use serde::{
    de::DeserializeOwned,
    Deserialize,
    Serialize
};

use serde_json_core::{
    de::from_slice,
    ser::to_string
};

use super::net;
use super::iir::IIR;

#[derive(Deserialize, Serialize)]
pub struct Request {
    pub channel: u8,
    pub iir: IIR,
}

#[derive(Serialize)]
pub struct Response<'a> {
    code: i32,
    message: &'a str,
}

#[derive(Serialize)]
pub struct Status {
    pub t: u32,
    pub x0: f32,
    pub y0: f32,
    pub x1: f32,
    pub y1: f32,
}

pub fn json_reply<T: Serialize>(socket: &mut net::socket::TcpSocket, msg: &T) {
    let mut u: String<U128> = to_string(msg).unwrap();
    u.push('\n').unwrap();
    socket.write_str(&u).unwrap();
}

pub struct Server {
    data: Vec<u8, U256>,
    discard: bool,
}

impl Server {
    pub fn new() -> Self {
        Self {
            data: Vec::new(),
            discard: false,
        }
    }

    pub fn poll<T, F, R>(
        &mut self,
        socket: &mut net::socket::TcpSocket,
        f: F,
    ) -> Option<R>
    where
        T: DeserializeOwned,
        F: FnOnce(&T) -> R,
    {
        while socket.can_recv() {
            let found = socket
                .recv(|buf| {
                    let (len, found) =
                        match buf.iter().position(|&c| c as char == '\n') {
                            Some(end) => (end + 1, true),
                            None => (buf.len(), false),
                        };
                    if self.data.len() + len >= self.data.capacity() {
                        self.discard = true;
                        self.data.clear();
                    } else if !self.discard && len > 0 {
                        self.data.extend_from_slice(&buf[..len]).unwrap();
                    }
                    (len, found)
                })
                .unwrap();
            if found {
                if self.discard {
                    self.discard = false;
                    json_reply(
                        socket,
                        &Response {
                            code: 520,
                            message: "command buffer overflow",
                        },
                    );
                    self.data.clear();
                } else {
                    let r = from_slice::<T>(&self.data[..self.data.len() - 1]);
                    self.data.clear();
                    match r {
                        Ok(res) => {
                            let r = f(&res);
                            json_reply(
                                socket,
                                &Response {
                                    code: 200,
                                    message: "ok",
                                },
                            );
                            return Some(r);
                        }
                        Err(err) => {
                            warn!("parse error {:?}", err);
                            json_reply(
                                socket,
                                &Response {
                                    code: 550,
                                    message: "parse error",
                                },
                            );
                        }
                    }
                }
            }
        }
        None
    }
}

