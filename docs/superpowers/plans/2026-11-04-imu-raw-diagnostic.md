# Passive bus IMU diagnostic implementation plan

> Execute inline. This is a local diagnostic preparation for the user's approved IMU comparison, not approval to change the live attitude algorithm or trigger motion.

**Goal:** Preserve the unfiltered, accepted ID 200 IMU sample alongside the same control-cycle telemetry, without another serial transaction or a policy-input change.

**Architecture:** Copy the decoded 12-byte core block and its board sequence/status at the existing successful `FeetechIo::read_sample` commit point. Store a monotonic host read-start time and transaction duration. Publish the fixed-size copy through `feetech_snapshot::Dynamic`, already used by `robot.rawSubscribe` and busStatus. Failed reads retain the last accepted sample with its original timestamp, and the surrounding `fresh_sample`/age fields mark stale data. Host/I2C IMUs omit this bus-specific diagnostic.

**Limits:** Host read-start is not the chip sample timestamp. Gyro and SFLP FIFO tags may have different ages. No accelerometer, fourth quaternion component, or chip timestamp is invented. The original packed quaternion and raw gyro remain in sensor coordinates. Metadata identifies board/protocol; consumers apply the correct scale and mounting rotation. No changes to decoding, median filters, calibration, target filters, torque or goals.

**Base:** Exact deployed runtime source `0791fcd44f71c705dc8bb0fbcde405593b52d19b`.

## Steps

- [x] Run existing `duck-control` tests on Linux using the existing host target cache.
- [x] Extend the serial fake-device regression to assert loss never retimestamps the raw sample and publishing snapshots sends no extra serial commands. Run it before implementation and observe failure on the missing raw diagnostic.
- [x] Add `BusImuRawSample` with `[u8; 12]`, sequence/status/age, monotonic read-start and transaction duration. Cache only after all sample validation. Add an optional `imu_raw` field to the native dynamic snapshot; preserve absent fields for host IMUs.
- [x] Bump the API version for the added native telemetry field and document that a diagnostic robotd build is required. No route refusal on version skew.
- [x] Run focused regression plus full `duck-control` tests; build/check `robotd` because it constructs snapshots. Verify an unchanged policy input and serial packet sequence under the fake device.
- [x] Prepare a bounded passive `robot.rawSubscribe` recorder that requires the new field and fails clearly on an old daemon. Retain cycle, raw diagnostic, gyro/quaternion/gravity and targets in their original binary framed record.
- [x] Record comparison outcome and remaining limitation. No live binary deployment or algorithm change in this patch preparation.
