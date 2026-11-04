use super::*;
use std::fs::File;
use std::os::fd::FromRawFd;

fn pty() -> (String, File, File) {
    let (mut master, mut slave) = (0, 0);
    let mut name = [0 as libc::c_char; 128];
    assert_eq!(
        unsafe {
            libc::openpty(
                &mut master,
                &mut slave,
                name.as_mut_ptr(),
                std::ptr::null(),
                std::ptr::null(),
            )
        },
        0
    );
    let name = unsafe { std::ffi::CStr::from_ptr(name.as_ptr()) }
        .to_str()
        .unwrap()
        .to_owned();
    (name, unsafe { File::from_raw_fd(master) }, unsafe {
        File::from_raw_fd(slave)
    })
}

#[test]
fn bus_verify_publishes_the_independent_contract() {
    let status = calibration_status(
        &Calibration::commissioning(JOINT_IDS),
        Path::new("/unused"),
        TrunkImuKind::Bus,
    );
    assert_eq!(status["imu_verification_contract"], "upright_nose_down_v1");
    assert_eq!(duck_ipc_proto::API_VERSION, 43);
}

#[test]
fn bus_verify_command_waits_for_real_tip_and_does_not_write() {
    let (port, _master, _slave) = pty();
    let mut bus = Bus::open(&port).unwrap();
    let dir =
        std::env::temp_dir().join(format!("microduck-bus-verify-route-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("calibration.toml");
    let mut calibration = Calibration::commissioning(JOINT_IDS);
    save_calibration(&path, &calibration).unwrap();
    let before = std::fs::read(&path).unwrap();
    let (reply, mut rx) = oneshot::channel();
    let mut verifying = None;
    process_job(
        Job {
            command: BusCommand::VerifyImu {
                confirmed: true,
                mount_wxyz: vec![1.0, 0.0, 0.0, 0.0],
            },
            reply,
        },
        &mut bus,
        &BTreeMap::new(),
        &mut calibration,
        &path,
        &mut None,
        &JOINT_IDS,
        &mut None,
        ImuContext {
            source: TrunkImuKind::Bus,
            reading: Some(SensorReading {
                quat: Some([1.0, 0.0, 0.0, 0.0]),
                gyro: [0.0; 3],
            }),
            sequence: Some(10),
        },
        &mut verifying,
        &mut None,
    );
    assert!(
        verifying.is_some(),
        "bus verification must wait for live nose-down samples"
    );
    assert!(matches!(
        rx.try_recv(),
        Err(oneshot::error::TryRecvError::Empty)
    ));
    assert_eq!(std::fs::read(&path).unwrap(), before);
    assert!(!calibration.imu_mount_verified);
    assert!(calibration.imu_bus_protocol.is_none());
    std::fs::remove_dir_all(path.parent().unwrap()).unwrap();
}

fn pending(now: Instant) -> PendingVerify {
    let (reply, _rx) = oneshot::channel();
    let upright = upright_check(
        TrunkImuKind::Bus,
        [1.0, 0.0, 0.0, 0.0],
        Some(SensorReading {
            quat: Some([1.0, 0.0, 0.0, 0.0]),
            gyro: [0.0; 3],
        }),
    )
    .unwrap();
    PendingVerify::new(reply, [1.0, 0.0, 0.0, 0.0], upright, now)
}

fn tipped(degrees: f64) -> SensorReading {
    let half = degrees.to_radians() / 2.0;
    SensorReading {
        quat: Some([half.cos(), 0.0, half.sin(), 0.0]),
        gyro: [0.0; 3],
    }
}

#[test]
fn bus_verify_rejects_missing_invalid_quaternion_instead_of_default_upright() {
    let identity = [1.0, 0.0, 0.0, 0.0];
    for quat in [
        None,
        Some([0.0; 4]),
        Some([f64::NAN, 0.0, 0.0, 0.0]),
        Some([f64::INFINITY, 0.0, 0.0, 0.0]),
        Some([2.0, 0.0, 0.0, 0.0]),
    ] {
        let sample = SensorReading {
            quat,
            gyro: [0.0; 3],
        };
        assert!(
            upright_check(TrunkImuKind::Bus, identity, Some(sample)).is_err(),
            "{sample:?}"
        );
        let now = Instant::now();
        assert!(
            pending(now)
                .observe_new(Some(sample), Some(1), now)
                .unwrap()
                .is_err()
        );
    }
}

#[test]
fn bus_verify_counts_distinct_samples_and_checks_timeout_even_after_a_tip() {
    let now = Instant::now();
    let mut check = pending(now);
    for _ in 0..10 {
        assert!(
            check
                .observe_new(Some(tipped(20.0)), Some(10), now)
                .is_none()
        );
    }
    assert_eq!(check.forward, 1);
    assert!(check.observe_new(None, Some(11), now).is_none());
    assert_eq!(
        check.forward, 1,
        "failed reads cannot reuse the preceding sample"
    );
    for seq in 11..14 {
        assert!(
            check
                .observe_new(Some(tipped(20.0)), Some(seq), now)
                .is_none()
        );
    }
    assert!(
        check
            .observe_new(Some(tipped(20.0)), Some(14), now)
            .unwrap()
            .is_ok()
    );

    let mut late = pending(now);
    for seq in 0..4 {
        assert!(
            late.observe_new(Some(tipped(20.0)), Some(seq), now)
                .is_none()
        );
    }
    assert!(
        late.observe_new(Some(tipped(20.0)), Some(4), now + NOSE_DOWN_WINDOW)
            .unwrap()
            .is_err(),
        "a fifth forward sample outside the window must not save"
    );
}

#[test]
fn bus_verify_full_q4_loop_uses_signed_raw_reading_and_only_saves_after_tip() {
    use duck_control::feetech::{READ, SYNC_READ, SYNC_WRITE, packet};
    use std::io::{Read, Write};
    use std::os::fd::AsRawFd;
    use std::sync::atomic::AtomicU8;

    struct Rig {
        state: Arc<RobotState>,
        stop: Arc<AtomicBool>,
        control: Option<std::thread::JoinHandle<()>>,
        device: Option<std::thread::JoinHandle<()>>,
    }
    impl Drop for Rig {
        fn drop(&mut self) {
            self.state.shutdown.store(true, Ordering::Relaxed);
            let control = self.control.take().unwrap().join();
            self.stop.store(true, Ordering::Relaxed);
            let device = self.device.take().unwrap().join();
            if !std::thread::panicking() {
                control.unwrap();
                device.unwrap();
            }
        }
    }

    let (port, mut wire, _slave) = pty();
    assert_ne!(
        unsafe { libc::fcntl(wire.as_raw_fd(), libc::F_SETFL, libc::O_NONBLOCK) },
        -1
    );
    let stop = Arc::new(AtomicBool::new(false));
    let pose = Arc::new(AtomicU8::new(0));
    let device_stop = stop.clone();
    let device_pose = pose.clone();
    let device = std::thread::spawn(move || {
        let mut pending = Vec::new();
        let mut bytes = [0u8; 512];
        let mut sequence = 0u32;
        while !device_stop.load(Ordering::Relaxed) {
            match wire.read(&mut bytes) {
                Ok(n) => pending.extend_from_slice(&bytes[..n]),
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {}
                other => panic!("PTY read {other:?}"),
            }
            while pending.len() >= 4 && pending.len() >= usize::from(pending[3]) + 4 {
                let len = usize::from(pending[3]) + 4;
                let tx: Vec<u8> = pending.drain(..len).collect();
                assert_eq!(tx[2..].iter().fold(0u8, |a, b| a.wrapping_add(*b)), 255);
                let (address, width) = (tx[5], usize::from(tx[6]));
                match tx[4] {
                    SYNC_WRITE => {
                        assert_eq!(address, 40, "verification never commands joint movement");
                        for row in tx[7..len - 1].chunks_exact(width + 1) {
                            assert_eq!(row[1], 0, "verification never enables torque");
                        }
                    }
                    READ => {
                        assert_eq!(tx[2], IMU_ID);
                        let data = match (address, width) {
                            (0, 5) => vec![1, 1, 0, 0x44, 0x4d],
                            (186, 8) => duck_control::full_q4::DESCRIPTOR.to_vec(),
                            (200, 28) => {
                                sequence = sequence.wrapping_add(1);
                                let mut raw = vec![0u8; 28];
                                // Negative-w samples prove the four signed components reach verification.
                                let q: [u16; 4] = match device_pose.load(Ordering::Relaxed) {
                                    0 => [0xbc00, 0, 0, 0],
                                    _ => [0xbbe1, 0, 0xb18f, 0],
                                };
                                for (i, h) in q.iter().enumerate() {
                                    raw[6 + i * 2..8 + i * 2].copy_from_slice(&h.to_le_bytes());
                                }
                                raw[14..18].copy_from_slice(&sequence.to_le_bytes());
                                raw[18..22].copy_from_slice(&(sequence * 8333).to_le_bytes());
                                raw[27] = 1;
                                raw
                            }
                            _ => panic!("unexpected IMU read {tx:?}"),
                        };
                        wire.write_all(&packet(IMU_ID, 0, &data).unwrap()).unwrap();
                    }
                    SYNC_READ => {
                        let mut replies = Vec::new();
                        assert_eq!(&tx[7..len - 1], JOINT_IDS);
                        for id in &tx[7..len - 1] {
                            let mut data = vec![0u8; width];
                            if address == 56 {
                                data[..2].copy_from_slice(&2048u16.to_le_bytes());
                                data[6] = 75;
                            } else {
                                assert_eq!((address, width), (40, 1));
                            }
                            replies.extend(packet(*id, 0, &data).unwrap());
                        }
                        wire.write_all(&replies).unwrap();
                    }
                    _ => panic!("unexpected request {tx:?}"),
                }
            }
            std::thread::sleep(Duration::from_micros(200));
        }
    });
    let dir =
        std::env::temp_dir().join(format!("microduck-bus-verify-full-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("calibration.toml");
    let mut cal = Calibration::commissioning(JOINT_IDS);
    cal.imu_mount_quat = [1.0, 0.0, 0.0, 0.0];
    save_calibration(&path, &cal).unwrap();
    let before = std::fs::read(&path).unwrap();
    let state = Arc::new(RobotState::new(
        &crate::Params::default(),
        Path::new("/unused"),
        false,
        false,
    ));
    state.commissioning.store(true, Ordering::Relaxed);
    let control_state = state.clone();
    let control_path = path.clone();
    let control = std::thread::spawn(move || {
        tokio::runtime::Builder::new_current_thread()
            .enable_time()
            .build()
            .unwrap()
            .block_on(run(
                &port,
                "feetech",
                "legacy",
                "full_q4_v1",
                &control_state,
                Duration::from_millis(20),
                &control_path,
                None,
            ));
    });
    let rig = Rig {
        state: state.clone(),
        stop,
        control: Some(control),
        device: Some(device),
    };
    let wait = |what: &str, predicate: &dyn Fn(&Value) -> bool| {
        let deadline = Instant::now() + Duration::from_secs(5);
        loop {
            let status = state
                .bus_snapshot
                .load_full()
                .map(|s| serde_json::to_value(s.as_ref()).unwrap())
                .unwrap_or_default();
            if predicate(&status) {
                break;
            }
            assert!(Instant::now() < deadline, "{what}: {status}");
            std::thread::sleep(Duration::from_millis(10));
        }
    };
    wait("valid full Q4", &|s| {
        s["last_sample"]["imu"]["valid"] == true
    });
    let verify = |mount: [f64; 4]| {
        let state = state.clone();
        std::thread::spawn(move || {
            serde_json::to_value(tokio::runtime::Runtime::new().unwrap().block_on(dispatch(
                &state,
                proto::Id::Number(1),
                json!({"action":"verify-imu",
                "confirmed":true,"mount_wxyz":mount}),
            )))
            .unwrap()
        })
    };
    let wrong = verify([0.0, 0.0, 0.0, 1.0]);
    wait("upright check", &|s| s["imu_verification"].is_string());
    assert_eq!(std::fs::read(&path).unwrap(), before);
    pose.store(1, Ordering::Relaxed);
    let reply = wrong.join().unwrap();
    assert!(
        reply["error"]["message"]
            .as_str()
            .unwrap()
            .contains("tail-down"),
        "{reply}"
    );
    assert_eq!(std::fs::read(&path).unwrap(), before);
    pose.store(0, Ordering::Relaxed);
    wait("return upright", &|s| {
        s["imu_verification"].is_null()
            && s["last_sample"]["imu"]["gravity"][0]
                .as_f64()
                .is_some_and(|x| x.abs() < 0.01)
    });
    let right = verify([1.0, 0.0, 0.0, 0.0]);
    wait("upright check", &|s| s["imu_verification"].is_string());
    assert_eq!(std::fs::read(&path).unwrap(), before);
    pose.store(1, Ordering::Relaxed);
    let reply = right.join().unwrap();
    assert!(
        reply["result"]["imu_check"]["nose_down"]["tilt_from_upright_deg"]
            .as_f64()
            .is_some_and(|x| (19.0..21.0).contains(&x)),
        "{reply}"
    );
    assert_eq!(reply["result"]["imu_check"]["upright"]["imu_source"], "bus");
    let saved = read_calibration(&path).unwrap();
    assert!(saved.imu_mount_verified);
    assert!(saved.imu_bus_protocol.is_some());
    assert!(!saved.motion_enabled);
    assert_eq!(
        serde_json::to_value(saved.joints).unwrap(),
        serde_json::to_value(cal.joints).unwrap()
    );
    drop(rig);
    std::fs::remove_dir_all(dir).unwrap();
}
