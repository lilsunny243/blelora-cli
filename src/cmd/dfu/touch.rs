use crate::{cmd::*, result};

use std::time::Duration;
use std::{thread, time};
use ferrous_serialport as serialport;

#[derive(Debug, StructOpt)]
/// Initiate Serial Touch
pub struct Cmd {
    /// Port to use
    #[structopt(long = "port", number_of_values(1))]
    port: String,

    /// Port to use
    #[structopt(long = "baud", number_of_values(1))]
    baud: u32,
}

impl Cmd {
    pub async fn run(self) -> result::Result {
        touch(&self.port, self.baud);

        // time to wait before closing
        thread::sleep(time::Duration::from_millis(1500));

        Ok(())
    }
}

fn touch(port: &String, baud: u32) {
    let _port = serialport::new(port, baud)
        .timeout(Duration::from_millis(10000))
        .open()
        .expect("Failed to open port");

    // time to wait open
    thread::sleep(time::Duration::from_millis(100));
}
