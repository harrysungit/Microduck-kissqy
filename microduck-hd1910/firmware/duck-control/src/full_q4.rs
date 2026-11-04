//! Opt-in signed, full quaternion wire format from the MD bus board.
use crate::imu::SensorReading;
use crate::io::{IoError, Result};
use std::time::Instant;

pub const DESCRIPTOR: [u8; 8] = [0x51, 0x34, 1, 1, 200, 28, 120, 0];
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sample {
    pub block: [u8; 28],
    pub sequence: u32,
    pub timestamp_us: u32,
    pub gyro_age_ms: u16,
    pub quat_age_ms: u16,
    pub status: u8,
    pub reading: SensorReading,
}
impl Sample {
    pub fn decode(raw: &[u8]) -> Result<Self> {
        let block: [u8; 28] = raw.try_into().map_err(|_| invalid("requires 200/28"))?;
        if block[27] != 1 {
            return Err(invalid("unknown block format"));
        }
        let q = std::array::from_fn::<_, 4, _>(|i| {
            crate::imu::half(u16::from_le_bytes([block[6 + i * 2], block[7 + i * 2]]))
        });
        let norm_sq: f64 = q.iter().map(|x| x * x).sum();
        if !q.iter().all(|x| x.is_finite()) || !(0.98..=1.02).contains(&norm_sq) {
            return Err(invalid("nonfinite or invalid quaternion norm"));
        }
        Ok(Self {
            block,
            sequence: u32::from_le_bytes(block[14..18].try_into().unwrap()),
            timestamp_us: u32::from_le_bytes(block[18..22].try_into().unwrap()),
            gyro_age_ms: u16::from_le_bytes(block[22..24].try_into().unwrap()),
            quat_age_ms: u16::from_le_bytes(block[24..26].try_into().unwrap()),
            status: block[26],
            reading: SensorReading {
                gyro: crate::imu::lsm6dsv16x_gyro(block[..6].try_into().unwrap()),
                quat: Some(q.map(|x| x / norm_sq.sqrt())),
            },
        })
    }
}
fn invalid(reason: &str) -> IoError {
    IoError::Bus(format!("full_q4_v1 IMU: {reason}"))
}
pub fn descriptor(raw: &[u8]) -> Result<()> {
    if raw != DESCRIPTOR {
        return Err(invalid("missing or unsupported descriptor at 186/8"));
    }
    Ok(())
}
#[derive(Default)]
pub struct Freshness {
    last: Option<(Sample, Instant, u64)>,
    /// A detected restart cannot heal itself by counting past the old values.
    reset_required: bool,
}
impl Freshness {
    pub fn check(&mut self, sample: &Sample, transaction_us: u64, now: Instant) -> Result<()> {
        if self.reset_required {
            return Err(invalid("counter/time continuity lost; reopen explicitly"));
        }
        // Readiness failure must not hide a decoded board restart.
        if let Some((last, _, _)) = &self.last {
            let sequence_delta = sample.sequence.wrapping_sub(last.sequence);
            let time_delta = sample.timestamp_us.wrapping_sub(last.timestamp_us);
            if sequence_delta == 0 {
                if time_delta != 0 || sample.block[6..14] != last.block[6..14] {
                    self.reset_required = true;
                    return Err(invalid("quaternion/time changed without a new sequence"));
                }
            } else if sequence_delta >= 0x8000_0000 || time_delta == 0 || time_delta >= 0x8000_0000
            {
                self.reset_required = true;
                return Err(invalid(
                    "counter/time moved backwards or board restarted; reopen explicitly",
                ));
            }
        }
        let age_us = u64::from(sample.gyro_age_ms.max(sample.quat_age_ms)) * 1000;
        let current_age = age_us.saturating_add(transaction_us);
        if sample.status != 0 || current_age > 50_000 {
            return Err(invalid("not ready or older than 50 ms"));
        }
        if let Some((last, accepted_at, accepted_age)) = &self.last {
            if sample.sequence == last.sequence {
                let elapsed = now
                    .saturating_duration_since(*accepted_at)
                    .as_micros()
                    .min(u64::MAX as u128) as u64;
                if accepted_age.saturating_add(elapsed) > 50_000 {
                    return Err(invalid("repeated sequence older than 50 ms"));
                }
                return Ok(());
            }
        }
        self.last = Some((*sample, now, current_age));
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::imu::{SflpDecoder, TrunkFrame};
    use std::time::Duration;

    fn bytes(q: [u16; 4], seq: u32, stamp: u32) -> [u8; 28] {
        let mut raw = [0; 28];
        for (i, h) in q.into_iter().enumerate() {
            raw[6 + i * 2..8 + i * 2].copy_from_slice(&h.to_le_bytes());
        }
        raw[14..18].copy_from_slice(&seq.to_le_bytes());
        raw[18..22].copy_from_slice(&stamp.to_le_bytes());
        raw[27] = 1;
        raw
    }
    fn hex(s: &str) -> Vec<u8> {
        (0..s.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
            .collect()
    }

    #[test]
    fn descriptor_and_wire_version_are_exact() {
        descriptor(&DESCRIPTOR).unwrap();
        for i in 0..8 {
            let mut d = DESCRIPTOR;
            d[i] ^= 1;
            assert!(descriptor(&d).is_err());
        }
        assert!(descriptor(&DESCRIPTOR[..7]).is_err());
        let mut raw = bytes([0x3c00, 0, 0, 0], 1, 1);
        raw[27] = 2;
        assert!(Sample::decode(&raw).is_err());
        assert!(Sample::decode(&raw[..27]).is_err());
    }
    #[test]
    fn c_serializer_fixture_decodes_all_fields_and_sensor_gravity() {
        let fixture: serde_json::Value = serde_json::from_str(include_str!(
            "../tests/fixtures/fullquat-wire-fixtures.json"
        ))
        .unwrap();
        descriptor(&hex(fixture["descriptor_hex"].as_str().unwrap())).unwrap();
        let start = Instant::now();
        let mut gate = Freshness::default();
        for (i, item) in fixture["samples"].as_array().unwrap().iter().enumerate() {
            let raw = hex(item["fullblock_hex"].as_str().unwrap());
            let sample = Sample::decode(&raw).unwrap();
            assert_eq!(sample.block.as_slice(), raw);
            assert_eq!(sample.sequence as u64, item["sequence"].as_u64().unwrap());
            assert_eq!(
                sample.timestamp_us as u64,
                item["timestamp_us"].as_u64().unwrap()
            );
            assert_eq!(
                sample.gyro_age_ms as u64,
                item["gyro_age_ms"].as_u64().unwrap()
            );
            assert_eq!(
                sample.quat_age_ms as u64,
                item["quat_age_ms"].as_u64().unwrap()
            );
            assert_eq!(sample.status as u64, item["status"].as_u64().unwrap());
            for k in 0..3 {
                let counts = i16::from_le_bytes(raw[k * 2..k * 2 + 2].try_into().unwrap());
                assert_eq!(i64::from(counts), item["gyro_raw"][k].as_i64().unwrap());
                assert!(
                    (sample.reading.gyro[k]
                        - f64::from(counts) * 0.0175 * std::f64::consts::PI / 180.0)
                        .abs()
                        < 1e-12
                );
            }
            for k in 0..4 {
                assert_eq!(
                    u16::from_le_bytes(raw[6 + k * 2..8 + k * 2].try_into().unwrap()) as u64,
                    item["quat_raw"][k].as_u64().unwrap()
                );
            }
            let q = sample.reading.quat.unwrap();
            assert_eq!(
                q,
                if i < 3 {
                    [-1.0, 0.0, 0.0, 0.0]
                } else {
                    [0.0, 1.0, 0.0, 0.0]
                }
            );
            if i == 4 {
                assert!(q[0].is_sign_negative());
            }
            let mut frame = TrunkFrame::new([1.0, 0.0, 0.0, 0.0]);
            frame.restart_median();
            assert_eq!(
                frame.push(sample.reading).gravity,
                if i < 3 {
                    [0.0, 0.0, -1.0]
                } else {
                    [0.0, 0.0, 1.0]
                }
            );
            assert_eq!(
                gate.check(&sample, 5000, start + Duration::from_millis(i as u64 * 8))
                    .is_ok(),
                i < 5
            );
        }
    }
    #[test]
    fn freshness_counts_both_ages_cycle_and_elapsed_repeated_sequence() {
        let now = Instant::now();
        let mut s = Sample::decode(&bytes([0x3c00, 0, 0, 0], 5, 5000)).unwrap();
        s.gyro_age_ms = 10;
        s.quat_age_ms = 4;
        let mut gate = Freshness::default();
        gate.check(&s, 20_000, now).unwrap();
        gate.check(&s, 1_000, now + Duration::from_millis(19))
            .unwrap();
        // Neither identical ages nor a smaller transaction can make the old sample younger.
        assert!(
            gate.check(&s, 1_000, now + Duration::from_millis(21))
                .is_err()
        );
        for (gyro, quat) in [(31, 0), (0, 31)] {
            s.gyro_age_ms = gyro;
            s.quat_age_ms = quat;
            assert!(Freshness::default().check(&s, 20_000, now).is_err());
        }
        for status in [1, 2, 3, 4, 128, 255] {
            s.status = status;
            assert!(Freshness::default().check(&s, 0, now).is_err());
        }
    }
    #[test]
    fn advancing_identical_q_wraps_but_reboots_and_backwards_time_are_rejected() {
        let now = Instant::now();
        let mut gate = Freshness::default();
        for (i, (seq, time)) in [
            (u32::MAX - 1, u32::MAX - 10000),
            (u32::MAX, u32::MAX - 1667),
            (0, 6666),
            (1, 14999),
        ]
        .into_iter()
        .enumerate()
        {
            gate.check(
                &Sample::decode(&bytes([0xbc00, 0, 0, 0], seq, time)).unwrap(),
                1000,
                now + Duration::from_millis(i as u64 * 8),
            )
            .unwrap();
        }
        for (seq, time) in [(0, 0), (2, 0), (0, 24000), (2, 14999)] {
            let mut gate = Freshness::default();
            gate.check(
                &Sample::decode(&bytes([0xbc00, 0, 0, 0], 1, 14999)).unwrap(),
                1000,
                now,
            )
            .unwrap();
            assert!(
                gate.check(
                    &Sample::decode(&bytes([0xbc00, 0, 0, 0], seq, time)).unwrap(),
                    1000,
                    now + Duration::from_millis(32)
                )
                .is_err()
            );
        }
    }
    #[test]
    fn detected_restart_stays_refused_even_after_counters_catch_up() {
        let now = Instant::now();
        let mut gate = Freshness::default();
        let before = Sample::decode(&bytes([0xbc00, 0, 0, 0], 100, 1_000_000)).unwrap();
        gate.check(&before, 1000, now).unwrap();
        let reboot = Sample::decode(&bytes([0xbc00, 0, 0, 0], 1, 8333)).unwrap();
        assert!(
            gate.check(&reboot, 1000, now + Duration::from_millis(20))
                .is_err()
        );
        let caught_up = Sample::decode(&bytes([0xbc00, 0, 0, 0], 130, 1_083_290)).unwrap();
        assert!(
            gate.check(&caught_up, 1000, now + Duration::from_secs(2))
                .is_err(),
            "a detected counter restart requires an explicit owner reopen"
        );
        Freshness::default()
            .check(&caught_up, 1000, now + Duration::from_secs(2))
            .unwrap();
    }

    #[test]
    fn invalid_status_regression_still_latches_restart() {
        let now = Instant::now();
        let mut gate = Freshness::default();
        gate.check(
            &Sample::decode(&bytes([0xbc00, 0, 0, 0], 100, 1_000_000)).unwrap(),
            1000,
            now,
        )
        .unwrap();
        let mut raw = bytes([0xbc00, 0, 0, 0], 1, 8333);
        raw[26] = 1;
        let reboot = Sample::decode(&raw).unwrap();
        assert!(
            gate.check(&reboot, 1000, now + Duration::from_millis(20))
                .is_err()
        );
        let caught_up = Sample::decode(&bytes([0xbc00, 0, 0, 0], 130, 1_083_290)).unwrap();
        assert!(
            gate.check(&caught_up, 1000, now + Duration::from_secs(2))
                .is_err(),
            "invalid status must not hide a decoded counter/time regression"
        );
        Freshness::default()
            .check(&caught_up, 1000, now + Duration::from_secs(2))
            .unwrap();
    }

    #[test]
    fn stale_regression_still_latches_restart() {
        let now = Instant::now();
        let mut gate = Freshness::default();
        gate.check(
            &Sample::decode(&bytes([0xbc00, 0, 0, 0], 100, 1_000_000)).unwrap(),
            1000,
            now,
        )
        .unwrap();
        let mut raw = bytes([0xbc00, 0, 0, 0], 101, 8333);
        raw[24..26].copy_from_slice(&51u16.to_le_bytes());
        let reboot = Sample::decode(&raw).unwrap();
        assert!(
            gate.check(&reboot, 1000, now + Duration::from_millis(20))
                .is_err()
        );
        let caught_up = Sample::decode(&bytes([0xbc00, 0, 0, 0], 130, 1_083_290)).unwrap();
        assert!(
            gate.check(&caught_up, 1000, now + Duration::from_secs(2))
                .is_err(),
            "stale age must not hide a decoded timestamp regression"
        );
        Freshness::default()
            .check(&caught_up, 1000, now + Duration::from_secs(2))
            .unwrap();
    }

    #[test]
    fn invalid_freshness_at_normal_wrap_does_not_latch_restart() {
        let now = Instant::now();
        for (status, age) in [(1, 0), (0, 51)] {
            let mut gate = Freshness::default();
            gate.check(
                &Sample::decode(&bytes([0xbc00, 0, 0, 0], u32::MAX, u32::MAX - 1667)).unwrap(),
                1000,
                now,
            )
            .unwrap();
            let mut raw = bytes([0xbc00, 0, 0, 0], 0, 6666);
            raw[26] = status;
            raw[22..24].copy_from_slice(&u16::to_le_bytes(age));
            assert!(
                gate.check(
                    &Sample::decode(&raw).unwrap(),
                    1000,
                    now + Duration::from_millis(8)
                )
                .is_err()
            );
            gate.check(
                &Sample::decode(&bytes([0xbc00, 0, 0, 0], 1, 14999)).unwrap(),
                1000,
                now + Duration::from_millis(16),
            )
            .unwrap();
        }
    }

    #[test]
    fn invalid_norm_and_nonfinite_are_rejected_before_normalization() {
        for q in [
            [0, 0, 0, 0],
            [0x3c00, 0x3c00, 0, 0],
            [0x7c00, 0, 0, 0],
            [0x7e00, 0, 0, 0],
            [0x3b00, 0, 0, 0],
        ] {
            assert!(Sample::decode(&bytes(q, 1, 1)).is_err());
        }
    }
    #[test]
    fn signed_near_zero_and_q_negation_preserve_truth_and_fast_tilt() {
        let mount = SflpDecoder::DEFAULT_MOUNT;
        let mut decoder = SflpDecoder::new(mount);
        let mut truth = TrunkFrame::new(mount);
        // Known binary16 values cross signed w=0; no reconstruction of w.
        for (q, known) in [
            ([0x211f, 0x3c00, 0, 0], [0.01, 1.0, 0.0, 0.0]),
            ([0, 0x3c00, 0, 0], [0.0, 1.0, 0.0, 0.0]),
            ([0xa11f, 0x3c00, 0, 0], [-0.01, 1.0, 0.0, 0.0]),
            ([0x3c00, 0, 0, 0], [1.0, 0.0, 0.0, 0.0]),
            ([0x3c00, 0, 0, 0], [1.0, 0.0, 0.0, 0.0]),
        ] {
            let s = Sample::decode(&bytes(q, 1, 1)).unwrap();
            let neg = Sample::decode(&bytes(q.map(|h| h ^ 0x8000), 1, 1)).unwrap();
            let mut a = TrunkFrame::new([1.0, 0.0, 0.0, 0.0]);
            a.restart_median();
            let mut b = TrunkFrame::new([1.0, 0.0, 0.0, 0.0]);
            b.restart_median();
            assert_eq!(a.push(s.reading).gravity, b.push(neg.reading).gravity);
            let actual = decoder
                .decode_sample(&crate::feetech::ImuSample::decode_full(&bytes(q, 1, 1)).unwrap());
            let expected = truth.push(SensorReading {
                gyro: [0.0; 3],
                quat: Some(known),
            });
            for k in 0..3 {
                assert!((actual.gravity[k] - expected.gravity[k]).abs() < 0.00001);
            }
            for k in 0..4 {
                assert!((actual.quat[k] - expected.quat[k]).abs() < 0.00001);
            }
            assert_eq!(actual.gyro, expected.gyro);
        }
        let end = decoder.decode_sample(
            &crate::feetech::ImuSample::decode_full(&bytes([0x3c00, 0, 0, 0], 1, 1)).unwrap(),
        );
        assert!(
            (end.gravity[0] + 1.0).abs() < 1e-12,
            "fast 180 degree tilt settles after the existing median only"
        );
    }
    #[test]
    fn physical_legacy_pair_remains_evidence_not_invented_full_ground_truth() {
        let pair = [
            [80, 255, 45, 8, 57, 255, 24, 171, 76, 57, 249, 185],
            [55, 255, 128, 8, 48, 255, 157, 42, 77, 185, 249, 57],
        ];
        let gravity = pair.map(|raw| {
            let mut f = TrunkFrame::new([1.0, 0.0, 0.0, 0.0]);
            f.restart_median();
            f.push(crate::imu::parse_block(&raw)).gravity
        });
        let dot = gravity[0]
            .iter()
            .zip(gravity[1])
            .map(|(a, b)| a * b)
            .sum::<f64>();
        assert!((dot.clamp(-1.0, 1.0).acos().to_degrees() - 4.563169).abs() < 1e-5);
    }
}
