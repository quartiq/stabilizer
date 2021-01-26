use super::NetworkStack;

use minimq::{QoS, Error, Property, MqttClient};

pub enum Action {
    Continue,
    Sleep,
    CommitSettings,
}

struct MqttInterface<T: miniconf::StringSet> {
    client: MqttClient<minimq::consts::U256, NetworkStack>,
    subscribed: bool,
    settings: T,
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
            client,
            subscribed: false,
            settings,
        }
    }

    pub fn current_settings(&self) -> &T {
        &self.settings
    }

    pub fn update(&mut self, time: u32) -> Result<bool, ()> {

        let sleep = self.client.network_stack.update(smoltcp::time::Instant::from_millis(time as i64));

        if !self.subscribed && self.client.is_connected().unwrap() {
            self.client.subscribe("stabilizer/settings/#", &[]);
            self.client.subscribe("stabilizer/commit", &[]);
        }

        let mut commit = false;

        match self.client.poll(|client, topic, message, properties| {
            let split = topic.split('/').iter();
            // TODO: Verify topic ID against our ID.
            let id = split.next().unwrap();

            // Process the command
            let command = split.next().unwrap();
            let response: String<consts::U512> = match command {
                "settings" => {
                    // Handle settings failures
                    let mut response: String<consts::U512> = String::new();
                    match self.settings.string_set(split.peekable(), message) {
                        Ok(_) => write!(&mut response, "{} written", topic).unwrap(),
                        Err(error) => {
                            write!(&mut response, "Settings failure: {}", error).unwrap();
                        }
                    };

                    response
                },
                "commit" => {
                    commit = true;
                    String::from("Committing pending settings");
                }
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
                self.client.publish(topic, &response.into_bytes(), QoS::AtMostOnce, &[]).unwrap();
            }
        }) {
            Ok(_) => {},
            Err(Error::Disconnected) => self.subscribed = false,
            Err(err) => error!("Unexpected error: {:?}", err)
        };

        let action = if commit {
            Action::Commit
        } else if sleep {
            Action::Sleep
        } else {
            Action::Continue
        };

        Ok(action)
    }
}
