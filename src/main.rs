use blelora::{
    cmd::{dfu, Opts},
    result::Result,
};
use log::LevelFilter;

use std::process;
use structopt::StructOpt;

#[derive(Debug, StructOpt)]
pub struct Cli {
    #[structopt(flatten)]
    opts: Opts,

    #[structopt(flatten)]
    cmd: Cmd,
}

#[derive(Debug, StructOpt)]
pub enum Cmd {
    Dfu(dfu::Cmd),
}

#[tokio::main]
async fn main() {
    env_logger::builder()
        .filter_level(LevelFilter::Info)
        .parse_default_env()
        .init();

    let cli = Cli::from_args();
    if let Err(e) = run(cli).await {
        log::error!("error: {:?}", e);
        process::exit(1);
    }
}

async fn run(cli: Cli) -> Result {
    match cli.cmd {
        Cmd::Dfu(cmd) => cmd.run().await,
    }
}
