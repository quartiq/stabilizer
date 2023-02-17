use miniconf::embedded_time;
use minimq::{
    embedded_nal::{IpAddr, TcpClientStack},
    Publication,
};

///! Stabilizer mqtt alarm listener

pub struct Alarm<Stack, Clock, const MESSAGE_SIZE: usize>
where
    Stack: TcpClientStack,
    Clock: embedded_time::Clock,
{
    mqtt: minimq::Minimq<Stack, Clock, MESSAGE_SIZE, 1>,
    subscribed: bool,
}

impl<Stack, Clock, const MESSAGE_SIZE: usize> Alarm<Stack, Clock, MESSAGE_SIZE>
where
    Stack: TcpClientStack,
    Clock: embedded_time::Clock + Clone,
{
    pub fn new(
        stack: Stack,
        client_id: &str,
        broker: IpAddr,
        clock: Clock,
    ) -> Result<Self, minimq::Error<Stack::Error>> {
        let mqtt =
            minimq::Minimq::new(broker, client_id, stack, clock.clone())?;
        Ok(Self {
            mqtt,
            subscribed: false,
        })
    }

    pub fn update(&mut self) -> Result<bool, minimq::Error<Stack::Error>> {
        if self.mqtt.client().is_connected() {
            if !self.subscribed {
                log::info!("alarm subed");
                self.subscribed = true;
                self.mqtt
                    .client()
                    .subscribe(&["topic".into()], &[])
                    .map(|_| false)
            } else {
                log::info!("alarm polling");
                self.mqtt
                    .poll(|client, topic, message, _properties| match topic {
                        "topic" => {
                            log::info!("{:?}", message);
                            let response = Publication::new(message)
                                .topic("echo")
                                .finish()
                                .unwrap();
                            client.publish(response).unwrap();
                            true
                        }
                        topic => {
                            log::info!("Unknown topic: {}", topic);
                            false
                        }
                    })
                    .map(|res| res.unwrap())
            }
        } else {
            self.subscribed = true;
            log::info!("alarm disconnected");
            Ok(false)
        }
    }
}
