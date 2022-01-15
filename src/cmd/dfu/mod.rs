use crate::{cmd::*, result::Result};

mod serial;
mod touch;

#[derive(Debug, StructOpt)]
/// Initiate DFU over serial
pub enum Cmd {
    Serial(serial::Cmd),
    Touch(touch::Cmd),
}

impl Cmd {
    pub async fn run(self) -> Result {
        match self {
            Self::Serial(cmd) => cmd.run().await,
            Self::Touch(cmd) => cmd.run().await,
        }
    }
}
