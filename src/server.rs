use heapless::{
    consts::*,
    String,
    Vec
};

use core::fmt::Write;


use serde::{
    Deserialize,
    Serialize
};

use serde_json_core::{
    de::from_slice,
    ser::to_string
};

use super::net;

#[derive(Deserialize, Serialize, Debug)]
pub enum Request<'a, 'b> {
    Read{attribute: &'a str},
    Write{attribute: &'a str, value: &'b str},
}

#[derive(Serialize)]
pub struct Response {
    code: i32,
    attribute: String<U128>,
    value: String<U128>,
}

impl Response {
    pub fn success<'a, 'b>(attribute: &'a str, value: &'b str) -> Self
    {
        Self { code: 200, attribute: String::from(attribute), value: String::from(value)}
    }

    pub fn error<'a, 'b>(attribute: &'a str, message: &'b str) -> Self
    {
        Self { code: 400, attribute: String::from(attribute), value: String::from(message)}
    }

    pub fn custom<'a>(code: i32, message : &'a str) -> Self
    {
        Self { code: code, attribute: String::from(""), value: String::from(message)}
    }
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

    pub fn poll<F>(
        &mut self,
        socket: &mut net::socket::TcpSocket,
        mut f: F,
    )
    where
        F: FnMut(&Request) -> Response
    {
        while socket.can_recv() {
            let found = socket.recv(|buf| {
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
            }).unwrap();

            if found {
                if self.discard {
                    self.discard = false;
                    json_reply(socket, &Response::custom(520, "command buffer overflow"));
                } else {
                    let r = from_slice::<Request>(&self.data[..self.data.len() - 1]);
                    match r {
                        Ok(res) => {
                            let response = f(&res);
                            json_reply(socket, &response);
                        },
                        Err(err) => {
                            warn!("parse error {:?}", err);
                            json_reply(socket, &Response::custom(550, "parse error"));
                        }
                    }
                }
                self.data.clear();
            }
        }
    }
}

