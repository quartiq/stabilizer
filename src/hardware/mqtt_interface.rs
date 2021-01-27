use super::NetworkStack;

use minimq::{
    embedded_nal::{IpAddr, Ipv4Addr},
    QoS, Error, Property, MqttClient
};
use heapless::{String, consts};
use core::fmt::Write;

pub enum Action {
    Continue,
    Sleep,
    CommitSettings,
}

pub struct MqttInterface<T: miniconf::StringSet> {
    client: core::cell::RefCell<MqttClient<minimq::consts::U256, NetworkStack>>,
    subscribed: bool,
    pub settings: core::cell::RefCell<T>,
}

impl<T> MqttInterface<T>
where
    T: miniconf::StringSet
{
    pub fn new(stack: NetworkStack, settings: T) -> Self {
        let client: MqttClient<minimq::consts::U256, _> = MqttClient::new(
                IpAddr::V4(Ipv4Addr::new(10, 0, 0, 1)),
                "stabilizer",
                stack).unwrap();

        Self {
            client: core::cell::RefCell::new(client),
            subscribed: false,
            settings: core::cell::RefCell::new(settings),
        }
    }

    pub fn update(&mut self, time: u32) -> Result<Action, ()> {

        let sleep = self.client.borrow_mut().network_stack.update(time);

        if !self.subscribed && self.client.borrow_mut().is_connected().unwrap() {
            self.client.borrow_mut().subscribe("stabilizer/settings/#", &[]).unwrap();
            self.client.borrow_mut().subscribe("stabilizer/commit", &[]).unwrap();
        }

        let mut commit = false;

        match self.client.borrow_mut().poll(|client, topic, message, properties| {
            let mut split = topic.split('/');
            // TODO: Verify topic ID against our ID.
            let _id = split.next().unwrap();

            // Process the command
            let command = split.next().unwrap();
            let response: String<consts::U512> = match command {
                "settings" => {
                    // Handle settings failures
                    let mut response: String<consts::U512> = String::new();
                    match self.settings.borrow_mut().string_set(split.peekable(), message) {
                        Ok(_) => write!(&mut response, "{} written", topic).unwrap(),
                        Err(error) => {
                            write!(&mut response, "Settings failure: {:?}", error).unwrap();
                        }
                    };

                    response
                },
                "commit" => {
                    commit = true;
                    String::from("Committing pending settings")
                },
                _ => String::from("Unknown topic"),
            };

            // Publish the response to the request over MQTT using the ResponseTopic property if
            // possible. Otherwise, default to a logging topic.
            if let Property::ResponseTopic(topic) = properties.iter().find(|&prop| {
                if let Property::ResponseTopic(_) = *prop {
                    true
                } else {
                    false
                }
            }).or(Some(&Property::ResponseTopic("stabilizer/log"))).unwrap() {
                client.publish(topic, &response.into_bytes(), QoS::AtMostOnce, &[]).unwrap();
            }
        }) {
            Ok(_) => {},
            Err(Error::Disconnected) => self.subscribed = false,
            Err(err) => error!("Unexpected error: {:?}", err)
        };

        let action = if commit {
            Action::CommitSettings
        } else if sleep {
            Action::Sleep
        } else {
            Action::Continue
        };

        Ok(action)
    }
}
