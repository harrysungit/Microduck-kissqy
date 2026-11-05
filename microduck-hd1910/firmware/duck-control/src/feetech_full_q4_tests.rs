use super::*;
use serialport::SerialPort;
use std::sync::{
    Arc, Mutex,
    atomic::{AtomicBool, AtomicU8, Ordering},
};

struct Peer {
    stop: Arc<AtomicBool>,
    fault: Arc<AtomicU8>,
    requests: Arc<Mutex<Vec<Vec<u8>>>>,
    thread: Option<std::thread::JoinHandle<()>>,
}
impl Drop for Peer {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        self.thread.take().unwrap().join().unwrap();
    }
}
fn peer(identity: [u8; 5], desc: [u8; 8], delay_ms: u64) -> (Bus, Peer) {
    let (mut device, mut host) = serialport::TTYPort::pair().unwrap();
    device.set_timeout(Duration::from_millis(2)).unwrap();
    host.set_timeout(Duration::from_millis(2)).unwrap();
    let stop = Arc::new(AtomicBool::new(false));
    let done = stop.clone();
    let fault = Arc::new(AtomicU8::new(0));
    let faults = fault.clone();
    let requests = Arc::new(Mutex::new(Vec::new()));
    let seen = requests.clone();
    let thread = std::thread::spawn(move || {
        let mut bytes = Vec::new();
        let mut next = [0];
        let mut sequence = 0u32;
        while !done.load(Ordering::Relaxed) {
            if device.read(&mut next).unwrap_or(0) == 0 {
                continue;
            }
            bytes.push(next[0]);
            if bytes.len() < 4 || bytes.len() < 4 + bytes[3] as usize {
                continue;
            }
            let tx = std::mem::take(&mut bytes);
            seen.lock().unwrap().push(tx.clone());
            let fault = faults.load(Ordering::Relaxed);
            if tx[4] == READ && tx[2] == IMU_ID {
                let raw = match tx[5] {
                    0 => identity.to_vec(),
                    186 => desc.to_vec(),
                    200 => {
                        if fault == 1 {
                            continue;
                        }
                        std::thread::sleep(Duration::from_millis(delay_ms));
                        sequence = sequence.wrapping_add(1);
                        let mut raw = vec![0; 28];
                        raw[7] = 0xbc;
                        raw[14..18].copy_from_slice(&sequence.to_le_bytes());
                        raw[18..22].copy_from_slice(&(sequence * 8333).to_le_bytes());
                        raw[27] = 1;
                        if fault == 3 {
                            raw[26] = 3;
                        }
                        if fault == 4 {
                            raw.pop();
                        }
                        raw
                    }
                    _ => panic!("unexpected IMU read {tx:?}"),
                };
                let mut wire = packet(IMU_ID, 0, &raw).unwrap();
                if fault == 5 && tx[5] == 200 {
                    *wire.last_mut().unwrap() ^= 1;
                }
                if fault == 6 && tx[5] == 200 {
                    wire.extend_from_slice(b"noise");
                }
                let _ = device.write_all(&wire);
            } else if tx[4] == SYNC_READ {
                std::thread::sleep(Duration::from_millis(delay_ms));
                let mut wire = Vec::new();
                for id in &tx[7..tx.len() - 1] {
                    if (fault == 2 || fault == 6) && *id == crate::model::JOINT_IDS[0] {
                        continue;
                    }
                    let mut raw = vec![0; tx[6] as usize];
                    if *id == IMU_ID {
                        raw[7] = 0x30;
                        raw[12] = sequence as u8;
                    } else if tx[5] == 56 {
                        raw[..2].copy_from_slice(&2048u16.to_le_bytes());
                        raw[6] = 73;
                        raw[7] = 35;
                    }
                    wire.extend(packet(*id, 0, &raw).unwrap());
                }
                let _ = device.write_all(&wire);
            } else {
                panic!("unexpected request {tx:?}");
            }
        }
    });
    (
        Bus {
            port: Box::new(host),
            timeout: Duration::from_millis(30),
            read_only: true,
            last_sync: SyncTrace::default(),
            imu_board: BusImuBoard::Ft6,
            imu_protocol: BusImuProtocol::Auto,
            imu_identity: None,
            imu_dxl2_sync: false,
            imu_profile: BusImuProfile::Legacy,
            observations: observations::Observations::default(),
            last_imu_response_at: None,
            full_q4: false,
            full_q4_verified: false,
        },
        Peer {
            stop,
            fault,
            requests,
            thread: Some(thread),
        },
    )
}
fn full_peer(delay: u64) -> (Bus, Peer) {
    let (mut bus, peer) = peer([1, 1, 0, 0x44, 0x4d], crate::full_q4::DESCRIPTOR, delay);
    bus.set_imu_quaternion("full_q4_v1").unwrap();
    bus.identify_imu().unwrap();
    (bus, peer)
}

#[test]
fn full_mode_requires_own_identity_and_exact_descriptor_never_falls_back() {
    for (identity, desc) in [
        (IMU_IDENTITY, crate::full_q4::DESCRIPTOR),
        ([1, 0, 0, 0x44, 0x4d], [0; 8]),
        ([1, 1, 0, 0x44, 0x4d], [0x51, 0x34, 2, 1, 200, 28, 120, 0]),
    ] {
        let (mut bus, _peer) = peer(identity, desc, 0);
        bus.set_imu_quaternion("full_q4_v1").unwrap();
        assert!(bus.identify_imu().is_err());
        assert!(bus.telemetry(&crate::model::JOINT_IDS).is_err());
    }
    let (mut bus, _peer) = peer(IMU_IDENTITY, [0; 8], 0);
    bus.set_imu_protocol("dxl2").unwrap();
    assert!(bus.set_imu_quaternion("full_q4_v1").is_err());
    assert!(bus.set_imu_quaternion("guess").is_err());
}

#[test]
fn full_mode_and_qinmo_profile_are_refused_in_both_selection_orders() {
    // These independent installation options must not combine into a wrong UART layout.
    let (mut bus, _peer) = peer(IMU_IDENTITY, [0; 8], 0);
    bus.set_imu_profile("qinmo").unwrap();
    assert!(bus.set_imu_quaternion("full_q4_v1").is_err());
    let (mut bus, _peer) = peer(IMU_IDENTITY, [0; 8], 0);
    bus.set_imu_quaternion("full_q4_v1").unwrap();
    assert!(bus.set_imu_profile("qinmo").is_err());
}

#[test]
fn full_mode_startup_observation_respects_descriptor_without_control_identity() {
    let (mut bus, device) = peer([1, 1, 0, 0x44, 0x4d], crate::full_q4::DESCRIPTOR, 0);
    bus.set_imu_quaternion("full_q4_v1").unwrap();
    let observation = bus.observe_startup_imu();
    assert!(observation.responding);
    assert_eq!(observation.sample_valid, Some(true));
    let info = observation.info.unwrap();
    assert_eq!(info.quaternion, "full_q4_v1");
    assert_eq!(info.descriptor, Some(crate::full_q4::DESCRIPTOR));
    assert!(bus.imu_info().is_none(), "diagnostics cannot establish control identity");
    assert!(!bus.full_q4_verified);
    let tx = device.requests.lock().unwrap();
    assert_eq!(tx.len(), 3);
    assert_eq!(tx[1], packet(IMU_ID, READ, &[186, 8]).unwrap());
    assert_eq!(tx[2], packet(IMU_ID, READ, &[200, 28]).unwrap());
}
#[test]
fn full_mode_reads_actual_block_then_servo_only_sync_in_single_owner() {
    let (mut bus, peer) = full_peer(0);
    let blocks = bus.telemetry(&crate::model::JOINT_IDS).unwrap();
    assert_eq!(blocks[&IMU_ID].len(), 28);
    assert_eq!(blocks.len(), 16);
    let tx = peer.requests.lock().unwrap();
    assert_eq!(tx.len(), 4);
    assert_eq!(tx[0], packet(IMU_ID, READ, &[0, 5]).unwrap());
    assert_eq!(tx[1], packet(IMU_ID, READ, &[186, 8]).unwrap());
    assert_eq!(tx[2], packet(IMU_ID, READ, &[200, 28]).unwrap());
    assert_eq!(
        tx[3],
        sync_read_packet(56, 15, &crate::model::JOINT_IDS).unwrap()
    );
    assert_eq!(bus.last_sync.rx_bytes, 34 + 15 * 21);
    assert!(bus.last_sync.missing_ids.is_empty());
}
#[test]
fn default_legacy_own_board_never_reads_descriptor_or_full_block() {
    let (mut bus, peer) = peer([1, 0, 0, 0x44, 0x4d], [0; 8], 0);
    bus.identify_imu().unwrap();
    bus.telemetry(&crate::model::JOINT_IDS).unwrap();
    let tx = peer.requests.lock().unwrap();
    assert_eq!(tx.len(), 2);
    assert_eq!(
        tx[1],
        sync_read_packet(
            56,
            15,
            &telemetry_ids(BusImuBoard::ImuToDxl, &crate::model::JOINT_IDS)
        )
        .unwrap()
    );
}
#[test]
fn full_mode_two_reads_share_twenty_ms_deadline() {
    let (mut bus, _peer) = full_peer(12);
    let start = Instant::now();
    assert!(bus.telemetry(&crate::model::JOINT_IDS).is_err());
    assert!(start.elapsed() < Duration::from_millis(35));
    assert!(bus.last_sync.deadline_exceeded);
}
#[test]
fn full_imu_failure_never_charges_unrequested_servos() {
    for fault in [1, 4, 5] {
        let (mut bus, peer) = full_peer(0);
        peer.fault.store(fault, Ordering::Relaxed);
        assert!(bus.telemetry(&crate::model::JOINT_IDS).is_err());
        assert_eq!(bus.last_sync.missing_ids, vec![IMU_ID]);
        assert_eq!((bus.last_sync.address, bus.last_sync.length), (56, 15));
        assert!(bus.last_sync.request_sent);
        assert_eq!(peer.requests.lock().unwrap().len(), 3);
    }
}
#[test]
fn full_partial_joint_recovery_is_only_allowed_for_clean_valid_full_read() {
    let (mut bus, peer) = full_peer(0);
    peer.fault.store(2, Ordering::Relaxed);
    let blocks = bus.telemetry_partial(&crate::model::JOINT_IDS).unwrap();
    assert_eq!(blocks.len(), 15);
    assert_eq!(bus.last_sync.missing_ids, vec![crate::model::JOINT_IDS[0]]);
    assert!(bus.last_sync.partial_received);
    assert!(bus.telemetry(&crate::model::JOINT_IDS).is_err());
    peer.fault.store(6, Ordering::Relaxed);
    assert!(bus.telemetry_partial(&crate::model::JOINT_IDS).is_err());
}

#[test]
fn full_io_uses_signed_decoder_and_retains_only_accepted_actual_raw_after_loss() {
    let (bus, peer) = full_peer(0);
    let cal = Calibration::commissioning(crate::model::JOINT_IDS);
    let mut reference = SflpDecoder::new(cal.imu_mount_quat);
    let mut io = FeetechIo::for_startup(bus, cal).unwrap();
    let before = serde_json::to_value(io.feetech_snapshot().unwrap().dynamic).unwrap();
    assert!(before.get("imu_full_raw").is_none());
    for i in 1..=26 {
        let sensors = io.read().unwrap();
        assert!(
            sensors.imu.quat[0] < 0.0,
            "signed w reaches the actual control decoder"
        );
        let dynamic = serde_json::to_value(io.feetech_snapshot().unwrap().dynamic).unwrap();
        assert!(
            dynamic.get("imu_raw").is_none(),
            "must not invent legacy xyz bytes"
        );
        let raw = &dynamic["imu_full_raw"];
        let block: Vec<u8> = serde_json::from_value(raw["block"].clone()).unwrap();
        assert_eq!(block.len(), 28);
        assert_eq!(raw["mode"], "full_q4_v1");
        assert_eq!(
            raw["descriptor"],
            serde_json::json!(crate::full_q4::DESCRIPTOR)
        );
        assert_eq!(raw["sequence"], i);
        assert_eq!(raw["read_index"], i);
        assert_eq!(
            sensors.imu,
            reference.decode_sample(&ImuSample::decode_full(&block).unwrap())
        );
    }
    assert!(io.imu_ready());
    let accepted =
        serde_json::to_value(io.feetech_snapshot().unwrap().dynamic).unwrap()["imu_full_raw"]
            .clone();
    for fault in [1, 3, 4, 5] {
        peer.fault.store(fault, Ordering::Relaxed);
        assert!(io.read().is_err());
        let after = serde_json::to_value(io.feetech_snapshot().unwrap().dynamic).unwrap();
        assert_eq!(
            after["imu_full_raw"], accepted,
            "failed read cannot retimestamp or change index"
        );
        assert!(io.joint_missing_total.iter().all(|n| *n == 0));
    }
    assert_eq!(io.telemetry.total_reads, 30);
    peer.fault.store(2, Ordering::Relaxed);
    assert!(io.read().is_ok());
    assert!(io.latest_partial);
    assert_eq!(io.partial_samples_total, 1);
    assert_eq!(io.joint_missing_total[0], 1);
    assert!(io.read_sample(false).is_err());
}
