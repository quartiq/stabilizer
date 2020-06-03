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
pub struct Response<'a, 'b> {
    code: i32,
    attribute: &'a str,
    value: &'b str,
}

impl<'a, 'b> Response<'a, 'b> {
    pub fn success<'c, 'd>(attribute: &'c str, value: &'d str) -> Self
    where
        'c: 'a,
        'd: 'b,
    {
        Self { code: 200, attribute: attribute, value: value}
    }

    pub fn error<'c, 'd>(attribute: &'c str, message: &'d str) -> Self
    where
        'c: 'a,
        'd: 'b,
    {
        Self { code: 400, attribute: attribute, value: message}
    }

    pub fn custom<'c>(code: i32, message : &'c str) -> Self
    where
        'c: 'b,
    {
        Self { code: code, attribute: "", value: message}
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

    pub fn poll<'a, 'b, F>(
        &mut self,
        socket: &mut net::socket::TcpSocket,
        f: F,
    )
    where
        F: FnOnce(&Request) -> Response<'a, 'b>
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
                    json_reply(socket, &Response::custom(520, "command buffer overflow"),
                    );
                    self.data.clear();
                } else {
                    let r = from_slice::<Request>(&self.data[..self.data.len() - 1]);
                    self.data.clear();
                    match r {
                        Ok(res) => {
                            let response = f(&res);
                            json_reply(socket, &response);
                            return;
                        },
                        Err(err) => {
                            warn!("parse error {:?}", err);
                            json_reply(socket, &Response::custom(550, "parse error"),
                            );
                        }
                    }
                }
            }
        }
    }
}

