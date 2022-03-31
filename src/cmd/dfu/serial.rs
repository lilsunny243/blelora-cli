use crate::{cmd::*, result};
use std::process;

use ferrous_serialport as serialport;
use serialport::{SerialPort, SerialPortType};
use std::time::{Duration, Instant};
use std::{thread, time};

use std::convert::TryInto;
use std::io::prelude::*;
use std::{fs, io, path::PathBuf};

use serde_json::Value;
use snailquote::unescape;
use zip::ZipArchive;

use indicatif::{ProgressBar, ProgressStyle};

use std::io::Cursor;

pub struct Firmware {
    pub firmware_dat_path: String,
    pub firmware_bin_path: String,
    pub sd_size: u64,
    pub bl_size: u64,
    pub application_size: u64,
    pub mode: u8,
}
pub struct Package {
    pub firmware: Firmware,
    pub manifest_json: Value,
}

impl Default for Firmware {
    fn default() -> Firmware {
        Firmware {
            firmware_dat_path: String::new(),
            firmware_bin_path: String::new(),
            bl_size: 0,
            sd_size: 0,
            application_size: 0,
            mode: 0,
        }
    }
}

impl Default for Package {
    fn default() -> Package {
        Package {
            firmware: Firmware::default(),
            manifest_json: Default::default(),
        }
    }
}
type Result<T> = std::result::Result<T, Box<dyn std::error::Error + Send + Sync>>;

static mut SEQUENCE_NUMBER: u32 = 0;

unsafe fn increment_sequence_number() {
    SEQUENCE_NUMBER = (SEQUENCE_NUMBER + 1) % 8;
}

const END: u8 = 0xC0;
const ESC: u8 = 0xDB;
const ESC_END: u8 = 0xDC;
const ESC_ESC: u8 = 0xDD;

const DATA_INTEGRITY_CHECK_PRESENT: u32 = 1;
const RELIABLE_PACKET: u32 = 1;
const HCI_PACKET_TYPE: u32 = 14;

const DFU_INIT_PACKET: u32 = 1;
const DFU_START_PACKET: u32 = 3;
const DFU_DATA_PACKET: u32 = 4;
const DFU_STOP_DATA_PACKET: u32 = 5;

const DFU_PACKET_MAX_SIZE: u32 = 512;

const USB_VID: u16 = 0x239a;
// const USB_PID: u16 = 0x002a;

#[allow(dead_code)]
const SOFTDEVICE: u8 = 1;
#[allow(dead_code)]
const BOOTLOADER : u8 = 2;
const SD_BL: u8 = 3;
const APPLICATION: u8 = 4;

#[derive(Debug, StructOpt)]
/// Initiate DFU transfer
pub struct Cmd {
    /// Package to use
    #[structopt(long = "package", number_of_values(1))]
    package: String,

    /// Optional port to use, overrides board selection
    #[structopt(long = "port", number_of_values(1))]
    port: Option<String>,

    /// Port to use
    #[structopt(long = "touch", number_of_values(1))]
    touch: Option<u32>,
}

impl Cmd {
    pub async fn run(self) -> result::Result {
        let mut package = unpack_package(&self.package).await;
        let unpacked_zip = std::path::Path::new("nrf_dfu/unpacked_zip/");

        let firmware_path: std::path::PathBuf = [
            unpacked_zip,
            std::path::Path::new(&package.firmware.firmware_bin_path),
        ]
        .iter()
        .collect();
        let firmware_path2: std::path::PathBuf = [
            unpacked_zip,
            std::path::Path::new(&package.firmware.firmware_bin_path),
        ]
        .iter()
        .collect();
        let firmware = fs::File::open(firmware_path).unwrap();
        let firmware_size = firmware.metadata().unwrap().len();
        let firmware_init_path: std::path::PathBuf = [
            unpacked_zip,
            std::path::Path::new(&package.firmware.firmware_dat_path),
        ]
        .iter()
        .collect();

        match serialport::available_ports() {
            Ok(ports) => {
                match ports.len() {
                    0 => log::debug!("No ports found."),
                    1 => log::debug!("Found 1 port:"),
                    n => log::debug!("Found {} ports:", n),
                };
                for p in ports {
                    log::debug!("  {}", p.port_name);
                    match p.port_type {
                        SerialPortType::UsbPort(info) => {
                            log::debug!("    Type: USB");
                            log::debug!("    VID:{:04x} PID:{:04x}", info.vid, info.pid);
                            log::debug!(
                                "     Serial Number: {}",
                                info.serial_number.as_ref().map_or("", String::as_str)
                            );
                            log::debug!(
                                "      Manufacturer: {}",
                                info.manufacturer.as_ref().map_or("", String::as_str)
                            );
                            log::debug!(
                                "           Product: {}",
                                info.product.as_ref().map_or("", String::as_str)
                            );
                        }
                        SerialPortType::BluetoothPort => {
                            log::debug!("    Type: Bluetooth");
                        }
                        SerialPortType::PciPort => {
                            log::debug!("    Type: PCI");
                        }
                        SerialPortType::Unknown => {
                            log::debug!("    Type: Unknown");
                        }
                    }
                }
            }
            Err(e) => {
                log::error!("Error listing serial ports: {0}", e);
            }
        }

        match self.touch {
            Some(x) => {
                log::debug!("TOUCH SERIAL PORT");
                touch(x).expect("Touch Failed");

                thread::sleep(time::Duration::from_millis(1500));
            }
            _ => log::debug!("no touch"),
        }

        let mut port = match self.port {
            Some(p) => {
                log::debug!("opening {}", &p);

                serialport::new(&p, 115200)
                    .timeout(Duration::from_millis(3000))
                    .open()?
            }
            _ => {
                let matching_ports: Vec<_> = serialport::available_ports()?
                    .into_iter()
                    .filter(|port| match &port.port_type {
                        serialport::SerialPortType::UsbPort(usb) => {
                            usb.vid == USB_VID //&& usb.pid == USB_PID
                        }
                        _ => false,
                    })
                    .collect();

                log::debug!("len: {}", matching_ports.len());

                if matching_ports.len() == 0 {
                    println!("The device could not be found. Please make sure your device is plugged in.");
                    process::exit(1);
                }

                log::debug!(
                    "opening {} (type {:?})",
                    &matching_ports[0].port_name,
                    matching_ports[0].port_type
                );
                serialport::new(&matching_ports[0].port_name, 115200)
                    .timeout(Duration::from_millis(3000))
                    .open()?
            }
        };

        println!("Starting Serial DFU");
        let mut bar = ProgressBar::new((firmware_size / 1000) * 2);

        bar.set_style(
            ProgressStyle::default_bar()
                .template("[{elapsed_precise}] {bar:60.cyan/blue} {percent}")
                .progress_chars("##-"),
        );

        if package.firmware.mode == APPLICATION {
            package.firmware.application_size = firmware_size;
        }

        log::debug!("Sending DFU start packet");
        send_start_dfu(&mut port, package.firmware.mode, package.firmware.sd_size, package.firmware.bl_size, package.firmware.application_size, &mut bar);

        thread::sleep(time::Duration::from_millis(2000));

        log::debug!("Sending DFU init packet");
        send_init_packet(&mut port, firmware_init_path, &mut bar);

        log::debug!("Sending firmware file");
        send_firmware(&mut port, firmware_path2, &mut bar);

        thread::sleep(time::Duration::from_millis(1000));

        bar.finish();

        println!("Complete");

        Ok(())
    }
}

fn touch(baud: u32) -> result::Result {
    let matching_ports: Vec<_> = serialport::available_ports()?
        .into_iter()
        .filter(|port| match &port.port_type {
            serialport::SerialPortType::UsbPort(usb) => {
                usb.vid == USB_VID //&& usb.pid == USB_PID
            }
            _ => false,
        })
        .collect();
    // log::debug!("len: {}", matching_ports.len());

    if matching_ports.len() == 0 {
        println!("The device could not be found. Please make sure your device is plugged in.");
        process::exit(1);
    }

    // This will error at the wrong baud, let it
    let _port = serialport::new(&matching_ports[0].port_name, baud)
        .timeout(Duration::from_millis(10000))
        .open();
    thread::sleep(time::Duration::from_millis(100));

    Ok(())
}

async fn fetch_url(url: String, file_name: String) -> Result<()> {
    if std::path::Path::new(&file_name).exists() {
        log::debug!("d_firmware.zip exist, removing");
        fs::remove_file(&file_name).unwrap();
    }

    let response = reqwest::get(url).await?;
    let mut file = std::fs::File::create(&file_name)?;
    let mut content = Cursor::new(response.bytes().await?);
    std::io::copy(&mut content, &mut file)?;
    Ok(())
}

async fn unpack_package(zip_file_path: &String) -> Package {
    if !std::path::Path::new("nrf_dfu").exists() {
        fs::create_dir("nrf_dfu").expect("Failed to create temp dir for package files");
    }

    let final_zip_file_path;
    let unpacked_zip_path = std::path::Path::new("nrf_dfu/unpacked_zip/");

    // figure out if package path is local or URL
    if zip_file_path.starts_with("http://") || zip_file_path.starts_with("https://") {
        log::debug!("Found URL String");
        // create directory for downloaded packages if it doesn't exist
        if !std::path::Path::new("nrf_dfu/downloaded_package").exists() {
            fs::create_dir("nrf_dfu/downloaded_package")
                .expect("Failed to create temp downloaded package dir for package zip");
        }

        fetch_url(
            zip_file_path.to_string(),
            "nrf_dfu/downloaded_package/d_firmware.zip".to_string(),
        )
        .await
        .unwrap();

        // set zip file path to newly downloaded zip path
        final_zip_file_path = "nrf_dfu/downloaded_package/d_firmware.zip".to_string();
    } else {
        log::debug!("Found file path");
        final_zip_file_path = zip_file_path.clone();
    }

    let zip_file = fs::File::open(final_zip_file_path).unwrap();
    let reader = io::BufReader::new(zip_file);
    let mut package = Package::default();

    let mut archive = ZipArchive::new(reader).unwrap();

    for i in 0..archive.len() {
        let mut file = archive.by_index(i).unwrap();
        let outpath = match file.enclosed_name() {
            Some(path) => path,
            None => {
                log::debug!("Entry {} has a suspicious path", file.name());
                continue;
            }
        };

        {
            let comment = file.comment();
            if !comment.is_empty() {
                log::debug!("Entry {} comment: {}", i, comment);
            }
        }

        if (&*file.name()).ends_with('/') {
            log::debug!(
                "Entry {} is a directory with name \"{}\"",
                i,
                outpath.display()
            );
        } else {
            log::debug!(
                "Entry {} is a file with name \"{}\" ({} bytes)",
                i,
                outpath.display(),
                file.size()
            );
        }
        if outpath.to_str() == Some("manifest.json") {
            log::debug!("manifest.json details:");
            let mut buffer = String::new();
            file.read_to_string(&mut buffer).unwrap();

            log::debug!("{}", buffer);
            let v: Value = serde_json::from_str(&buffer).unwrap();
            // log::debug!("{}", v["manifest"]);
            package.manifest_json = v.to_owned();
            if !v["manifest"]["application"].is_null() {
                log::debug!("Application Firmware Found");

                package.firmware.firmware_dat_path =
                    unescape(&v["manifest"]["application"]["dat_file"].to_string()).unwrap();
                package.firmware.firmware_bin_path =
                    unescape(&v["manifest"]["application"]["bin_file"].to_string()).unwrap();
                package.firmware.mode = APPLICATION;
            } else if !v["manifest"]["softdevice_bootloader"].is_null() {
                log::debug!("Bootloader Firmware Found");

                package.firmware.firmware_dat_path =
                    unescape(&v["manifest"]["softdevice_bootloader"]["dat_file"].to_string())
                        .unwrap();
                package.firmware.firmware_bin_path =
                    unescape(&v["manifest"]["softdevice_bootloader"]["bin_file"].to_string())
                        .unwrap();
                package.firmware.sd_size =
                    v["manifest"]["softdevice_bootloader"]["sd_size"].as_u64().unwrap();
                package.firmware.bl_size =
                    v["manifest"]["softdevice_bootloader"]["bl_size"].as_u64().unwrap();
                package.firmware.mode = SD_BL;
            }
        }
    }

    for i in 0..archive.len() {
        let mut file = archive.by_index(i).unwrap();
        let file_outpath = match file.enclosed_name() {
            Some(path) => path,
            None => continue,
        };

        let outpath: PathBuf = [unpacked_zip_path, file_outpath].iter().collect();

        if (&*file.name()).ends_with('/') {
            log::debug!("File {} extracted to \"{}\"", i, outpath.display());
            fs::create_dir_all(&outpath).unwrap();
        } else {
            log::debug!(
                "File {} extracted to \"{}\" ({} bytes)",
                i,
                outpath.display(),
                file.size()
            );
            if let Some(p) = outpath.parent() {
                if !p.exists() {
                    fs::create_dir_all(&p).unwrap();
                }
            }
            let mut outfile = fs::File::create(&outpath).unwrap();
            io::copy(&mut file, &mut outfile).unwrap();
        }

        // Get and Set permissions
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;

            if let Some(mode) = file.unix_mode() {
                fs::set_permissions(&outpath, fs::Permissions::from_mode(mode)).unwrap();
            }
        }
    }

    return package;
}

fn send_start_dfu(
    port: &mut Box<dyn SerialPort>,
    program_mode: u8,
    sd_size: u64,
    bl_size: u64,
    application_size: u64,
    bar: &mut ProgressBar,
) {
    let mut buf = DFU_START_PACKET.to_ne_bytes().to_vec();
    buf.extend((program_mode as u32).to_ne_bytes());
    buf.extend((sd_size as u32).to_ne_bytes());
    buf.extend((bl_size as u32).to_ne_bytes());
    buf.extend((application_size as u32).to_ne_bytes());

    let packet = hci_packet(buf);
    send_packet(port, packet, bar);
}

fn send_init_packet(
    port: &mut Box<dyn SerialPort>,
    firmware_init_path: std::path::PathBuf,
    bar: &mut ProgressBar,
) {
    let mut file_data = Vec::new();
    let mut firmware_init = fs::File::open(firmware_init_path).unwrap();
    firmware_init.read_to_end(&mut file_data).unwrap();

    let mut buf = DFU_INIT_PACKET.to_ne_bytes().to_vec();
    // buf.extend(DFU_INIT_PACKET.to_ne_bytes());
    // log::debug!("init_file_data: {:x?}", file_data);
    buf.extend(file_data);
    buf.extend(vec![0x00, 0x00]);

    let packet = hci_packet(buf);
    send_packet(port, packet, bar);
}

fn send_firmware(
    port: &mut Box<dyn SerialPort>,
    firmware_path: std::path::PathBuf,
    bar: &mut ProgressBar,
) {
    let mut firmware = fs::File::open(firmware_path).unwrap();
    let firmware_size = firmware.metadata().unwrap().len();

    let mut firmware_data = Vec::new();
    firmware.read_to_end(&mut firmware_data).unwrap();

    // log::debug!("firmware: {:x?}", firmware_data);

    let mut frames: Vec<Vec<u8>> = vec![];

    log::debug!("firmware_size: {}", firmware_size);

    for i in (0..firmware_size).step_by(DFU_PACKET_MAX_SIZE.try_into().unwrap()) {
        // log::debug!("{}", i);
        let mut frame = vec![];
        frame.extend(DFU_DATA_PACKET.to_ne_bytes());
        let start: usize = i.try_into().unwrap();
        let mut end: usize = (i + DFU_PACKET_MAX_SIZE as u64).try_into().unwrap();
        if end as u64 > firmware_size {
            end = (i + (firmware_size - i)).try_into().unwrap();
        }
        // log::debug!("start: {} end: {}", start, end);

        let data = firmware_data[start..end].to_vec();
        frame.extend(data);
        // log::debug!("frame length: {}", frame.len());
        let packet = hci_packet(frame);
        frames.extend(vec![packet]);
    }
    let frame_count = frames.len();

    // log::debug!("{:?}", frames);
    log::debug!("frame_count: {}", frame_count);

    let mut frame_count_sent = 0;
    let mut count = 0;

    for frame in frames {
        // log::debug!("frame: {:x?}", frame);
        send_packet(port, frame, bar);
        frame_count_sent += 1;
        log::debug!("frame_count_sent: {}", frame_count_sent);
        if count % 8 == 0 {
            thread::sleep(time::Duration::from_millis(150));
        }
        count += 1;
    }

    thread::sleep(time::Duration::from_millis(150));

    send_stop_data_dfu(port, bar);
}

fn send_stop_data_dfu(port: &mut Box<dyn SerialPort>, bar: &mut ProgressBar) {
    log::debug!("Sending Stop Data DFU");
    let buf = DFU_STOP_DATA_PACKET.to_ne_bytes().to_vec();
    let packet = hci_packet(buf);
    send_packet(port, packet, bar);
}

fn send_packet(port: &mut Box<dyn SerialPort>, frame: Vec<u8>, bar: &mut ProgressBar) {
    let mut attempts = 0;
    let mut last_ack: Option<u8> = None;
    let mut packet_sent = false;

    // log::debug!("frame: {:?}", frame);

    while !packet_sent {
        // log::debug!("PC -> target: {:x?}", frame);
        port.write(&frame).expect("send packet failure");
        let now = Instant::now();
        attempts += 1;

        let ack = get_ack_nr(port);

        let new_now = Instant::now();
        log::debug!("{:?}", new_now.duration_since(now));

        log::debug!("ack: {:?}", ack);

        bar.inc(1);

        if last_ack == None {
            break;
        }

        if ack.unwrap() == (last_ack.unwrap() + 1) % 8 {
            last_ack = ack;
            packet_sent = true;

            if attempts > 3 {
                println!("The firmware update failed. Please try again.");
                process::exit(1);
            }
        }
    }
}

fn get_ack_nr(port: &mut Box<dyn SerialPort>) -> Option<u8> {
    let mut esc_count = 0;
    let mut temp_buf: Vec<u8> = vec![];

    while esc_count < 2 {
        let mut serial_buf: Vec<u8> = vec![0; 6];

        match port.read(serial_buf.as_mut_slice()) {
            Ok(_) => log::debug!("read successful"),
            Err(_) => return None,
        }
        temp_buf.extend(serial_buf);

        // log::debug!("serial_buf: {:x?}", temp_buf);

        esc_count += temp_buf.iter().filter(|&n| *n == 0xC0).count();
        // log::debug!("esc_count: {}", esc_count);
    }

    // log::debug!("PC <- target: {:x?}", temp_buf);

    let mut new_buf = Vec::new();
    decode_frame(&temp_buf[1..], &mut new_buf).unwrap();

    // log::debug!("PC <- target: {:x?}", new_buf);

    return Some((new_buf[0] >> 3) & 0x07);
}

fn hci_packet(frame: Vec<u8>) -> Vec<u8> {
    unsafe {
        increment_sequence_number();
        let header = slip_parts_to_four_bytes(
            SEQUENCE_NUMBER,
            DATA_INTEGRITY_CHECK_PRESENT,
            RELIABLE_PACKET,
            HCI_PACKET_TYPE,
            frame.len() as u32,
        );

        // log::debug!("Slip Bytes: {:x?}", header);

        let mut data = Vec::new();
        data.extend(header);
        data.extend(frame);

        let crc16 = calc_crc16(data.to_owned());

        let temp1 = (crc16 & 0xFF) as u8;
        data.extend(vec![temp1]);
        let temp2 = ((crc16 & 0xFF00) >> 8) as u8;
        data.extend(vec![temp2]);

        let mut new_buf = Vec::new();
        encode_frame(&data, &mut new_buf).unwrap();

        let mut packet: Vec<u8> = vec![0xC0];
        packet.extend(new_buf);
        return packet;
    }
}

fn slip_parts_to_four_bytes(seq: u32, dip: u32, rp: u32, pkt_type: u32, pkt_len: u32) -> Vec<u8> {
    let mut buf = vec![0x00, 0x00, 0x00, 0x00];
    buf[0] = seq | (((seq + 1) % 8) << 3) | (dip << 6) | (rp << 7);
    buf[1] = pkt_type | ((pkt_len & 0x000F) << 4);
    buf[2] = (pkt_len & 0x0FF0) >> 4;
    buf[3] = (!(buf[..3].iter().sum::<u32>()) + 1) & 0xFF;

    // log::debug!("slip_buf {:?}", buf);

    let mut vec8: Vec<u8> = vec![0x00, 0x00, 0x00, 0x00];
    vec8[0] = buf[0] as u8;
    vec8[1] = buf[1] as u8;
    vec8[2] = buf[2] as u8;
    vec8[3] = buf[3] as u8;

    // log::debug!("slip_buf {:?}", vec8);

    return vec8;
}

fn calc_crc16(vec_data: Vec<u8>) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for b in vec_data {
        crc = (crc >> 8 & 0x00FF) | (crc << 8 & 0xFF00);
        crc = crc ^ (b as u16);
        crc = crc ^ (crc & 0x00FF) >> 4;
        crc = crc ^ (crc << 8) << 4;
        crc = crc ^ ((crc & 0x00FF) << 4) << 1;
    }

    // log::debug!("crc: {}", crc);

    return crc;
}

fn encode_frame(buf: &[u8], mut writer: impl Write) -> io::Result<()> {
    for &byte in buf {
        match byte {
            END => writer.write_all(&[ESC, ESC_END])?,
            ESC => writer.write_all(&[ESC, ESC_ESC])?,
            _ => writer.write_all(&[byte])?,
        }
    }

    writer.write_all(&[END])?;

    Ok(())
}

fn decode_frame(reader: impl Read, buf: &mut Vec<u8>) -> io::Result<()> {
    let mut bytes = reader.bytes();
    // log::debug!("decode_frame bytes: {:?}", bytes.next());
    loop {
        let encoded_byte = match bytes.next() {
            Some(byte) => byte,
            None => return Err(io::ErrorKind::UnexpectedEof.into()),
        };

        let decoded_byte = match encoded_byte? {
            ESC => match bytes.next() {
                None => return Err(io::ErrorKind::UnexpectedEof.into()),
                Some(Ok(ESC_ESC)) => ESC,
                Some(Ok(ESC_END)) => END,
                Some(Ok(invalid)) => {
                    return Err(io::Error::new(
                        io::ErrorKind::InvalidData,
                        format!("invalid byte following ESC: 0x{:02x}", invalid),
                    ))
                }
                Some(Err(e)) => return Err(e),
            },
            END => return Ok(()),
            other => other,
        };

        buf.push(decoded_byte);
    }
}
