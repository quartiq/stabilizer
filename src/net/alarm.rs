use heapless::String;
use miniconf::embedded_time;
use minimq::{
    embedded_nal::{IpAddr, TcpClientStack},
    Publication,
};
use stm32h7xx_hal::device::iwdg::pr;

/// Stabilizer mqtt alarm listener
pub struct Alarm<
    Stack,
    Clock,
    const MESSAGE_SIZE: usize,
    const MAX_TOPIC_LENGTH: usize,
> where
    Stack: TcpClientStack,
    Clock: embedded_time::Clock,
{
    mqtt: minimq::Minimq<Stack, Clock, MESSAGE_SIZE, 1>,
    subscribed: bool,
    prefix: String<MAX_TOPIC_LENGTH>,
}

impl<
        Stack,
        Clock,
        const MESSAGE_SIZE: usize,
        const MAX_TOPIC_LENGTH: usize,
    > Alarm<Stack, Clock, MESSAGE_SIZE, MAX_TOPIC_LENGTH>
where
    Stack: TcpClientStack,
    Clock: embedded_time::Clock + Clone,
{
    pub fn new(
        stack: Stack,
        client_id: &str,
        prefix: &str,
        broker: IpAddr,
        clock: Clock,
    ) -> Result<Self, minimq::Error<Stack::Error>> {
        let mqtt =
            minimq::Minimq::new(broker, client_id, stack, clock.clone())?;
        let mut prefix = String::from(prefix);
        prefix.push_str("/alarm").unwrap();
        Ok(Self {
            mqtt,
            subscribed: false,
            prefix,
        })
    }

    /// Update the alarm mqtt interface. Returns Ok(false) if the alarm was renewed.
    /// (aka if true has been published on the alarm topic)
    /// Returns Ok(true) if the alarm is set.
    pub fn update(&mut self) -> Result<bool, minimq::Error<Stack::Error>> {
        if !self.subscribed && self.mqtt.client().is_connected() {
            log::info!("alarm subed");
            self.subscribed = true;
            self.mqtt
                .client()
                .subscribe(&[(self.prefix.as_str()).into()], &[])
                .map(|_| false)
        } else {
            self.mqtt
                .poll(|client, topic, message, _properties| match topic {
                    // Todo: get this to use prefix from outside closure
                    prefix => {
                        log::info!("{:?}", message);
                        let response = Publication::new(message)
                            .topic("echo")
                            .finish()
                            .unwrap();
                        client.publish(response).unwrap();
                        true
                    }
                })
                .map(|res| {
                    // log::info!("res: {:?}", res);
                    false
                })
        }
    }
}
