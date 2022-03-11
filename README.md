# BLELoRa CLI 
A command-line utility written in Rust that supports Device Firmware Updates (DFU) for devices running [Adafruit_nRF52_Bootloader](https://github.com/adafruit/Adafruit_nRF52_Bootloader).

## Device Support
Currently the [WisBlock-RAK4631](https://store.rakwireless.com/products/rak4631-lpwan-node) is the only supported board. The option to specify additional supported boards will be added soon.

## Install 
For **macOS(Intel & M1)** users, install [brew](https://brew.sh/) first, then use the install command below:
```
brew install blelora/blelora-cli/blelora
```

For **Windows** users, install [Scoop](https://scoop.sh/) first, then use the install command below:
```
scoop install https://raw.githubusercontent.com/blelora/blelora-cli/master/install/blelora.json
```

From **Pre-compiled Binaries**:  
Download from the [releases](https://github.com/blelora/blelora-cli/releases) page.

With **Cargo**, the Rust package manager:  
```
cargo install blelora
```
## Usage
```
$ blelora --help 
blelora 0.1.0
Common options

USAGE:
    blelora <SUBCOMMAND>

FLAGS:
    -h, --help       Prints help information
    -V, --version    Prints version information

SUBCOMMANDS:
    dfu     Initiate DFU over serial
    help    Prints this message or the help of the given subcommand(s)
```
Upload Firmware over DFU Serial
```
$ blelora dfu serial --package <zip file location or URL to zip> --touch 1200
```

## Roadmap

 * [x] DFU Capability
 * [ ] LoRaWAN Credential Configuration
