# duck-658d IMU comparison result — 2026-11-04

## Decision

Do not deploy the candidate gravity correction or an extra gravity low-pass. The offline experiment is complete, but it has not demonstrated an improvement in physical walking. Robot calibration, model and target filters remain unchanged; the latest readback confirmed policy disabled, torque disabled, head 0.5 and legs 0.7.

The fixed experiment and full results are in `artifacts/bus-duck-check-20261104/imu-replay-32` in the parent workspace. `comparison.json` is the machine-readable result; `experiment.json` records the constants fixed before replay.

## Evidence and limits

| Recorded segment | Current maximum adjacent gravity-vector change | Gyro-aided candidate |
|---|---:|---:|
| HOME-standing board rotation | 1.857° | 0.459° |
| Supported/lying rotation | 0.182° | 0.182° |
| Higher-speed walk | 7.141° | 9.873° |
| Lower-speed walk | 8.036° | 2.090° |

These are changes between samples, not errors against independently measured physical angles. The faster-walk candidate has a larger transition discontinuity, so improvement on the turntable alone is insufficient.

In the known-truth synthetic 1 Hz pitch test, the current median has a best-fit delay of 20 ms. Adding a 0.1 s low-pass raises this to 100 ms. The candidate is 20 ms with unfiltered gyro and 40 ms with the already-median-filtered gyro available in present recordings. These are simulation results, not measured robot recovery latency.

The previous standing rotation returned within 0.352° of its initial gravity vector. This does not show large accumulating tilt drift in that test. Local jumps close to reconstructed sensor quaternion w=0 remain a credible precision problem, but their contribution to walking forward lean or left turns is not yet established.

## Passive diagnostic patch

Branch `diag/imu-raw-20261104` is based on the exact deployed source `0791fcd44f71c705dc8bb0fbcde405593b52d19b`. It copies the existing accepted 12-byte IMU block, board sequence/status/age, host read-start time, read index and transaction duration into native bus/raw telemetry. No new serial transaction or policy input is added. Host time is explicitly not a chip timestamp; the fast block contains neither accelerometer nor a fourth quaternion component.

`scripts/capture_bus_imu_raw.py` collects bounded original r17 frames using only `robot.busStatus` and `robot.rawSubscribe`. It refuses old runtimes missing `native.imu_raw`; it does not infer unfiltered gyro from a filtered value. Captures retain the original metadata and same-cycle state/bus records.

The ARM64 diagnostic runtime `0791fcd-feetech-imu-rawdiag.1` was temporarily installed on duck-658d while unloaded. Its SHA-256 is `a2c440779967009c1930e9d5a2ee3429745ee415e30d44391b902ada730e11cc`. The original executable remained untouched; a runtime-only systemd override selected the diagnostic executable with `--no-policy`, and padd/duck-echo were temporarily masked. No firmware flashing, torque-on or movement was triggered. Calibration, execution profile and robot configuration hashes were checked unchanged. **After collection the original runtime and previously active controller services were restored and verified**, as detailed below.

The first 12-second recording (`static-33.raw`, SHA-256 `c9787050f79cf38424f3ad809259108b61275c6d4c955d8a24954b6730f40dec`) contains 600 dynamic samples with no read-index or control-cycle gaps and no new failed reads. Every frame confirms torque off and model off. Re-decoding raw bytes reproduces the published quaternion within 2.3e-14 degrees, and the 598 complete median windows reproduce gravity and gyro within floating-point roundoff. Thus the recorder retains the same-cycle inputs correctly. This lying/static pose does not expose the near-w=0 condition (minimum sensor w=0.951); it is a logging check, not proof that walking is fixed. Static body gyro median remains approximately [0.102, 0.661, 0.065] degrees/s; the sensor's uncorrected gyro and its internally fused attitude must not be conflated.

## Verification

- Linux `duck-control`: 251 passed. Eight pre-existing ONNX integration tests are explicitly ignored by the suite, requiring its ONNX Runtime setup.
- The extended fake-serial test first failed because the raw field was absent, then passed. It verifies original bytes, unchanged decoder output, and no old-sample retimestamp after loss.
- Host IMU tests verify the bus-specific raw field is absent. The existing snapshot equivalence/no-extra-UART test passes.
- Linux recorder tests: 4 passed, including fragmented frames, invalid size/truncation, rejection of old runtime, exact byte retention and an actual Unix-socket session with only the two read routes.
- Rust raw-stream fixture → existing Python decoder: exact original IMU bytes and same-cycle state survived serialization. The targeted raw-stream test passed; `raw-fixture-verification.txt` records the cross-language check.
- Broader `robotd raw_` checks: 19 passed, 2 failed both before and after this patch. Exact-base rerun reproduced the startup fallback expectation and acquisition-count expectation failures; neither is presented as fixed. Logs are retained as `robotd-baseline-tests.txt` and `robotd-tests.txt`.

## Next controlled comparison

After the passive diagnostic runtime is validated and installed while unloaded, first collect static data and one supported rotation without model control. Confirm raw byte decoding, sequence progression, host timing and the source of the near-w=0 jump. Re-run offline comparisons using unfiltered gyro. Only a candidate that also follows real rapid tilt without a transition jump should proceed to a short, operator-controlled physical comparison. Do not change the leg filter, mounting calibration and attitude algorithm together.

## Physical capture and restoration

- `rotation-34.raw` has 3,750 same-cycle frames over 75 seconds; SHA-256 `07c4533f6b3aaabcacb95473bad5aded76fd4f24f9d931ebba91d88e5b68c1fb`. It captures the operator's lying rotation: measured tilt remains 94.57–95.29 degrees. There are no read/cycle gaps or new failed reads. Before/after average gravity differs by 0.068 degrees; maximum adjacent raw gravity change is 0.156 degrees. Only four samples reach sensor w < 0.05, so this is not a strong test of the upright precision singularity.
- The operator reported also making an upright turn. It was not present in that 75-second recording. A subsequent live sample showed approximately 11 degrees tilt, demonstrating that the pose had changed after the captured lying interval. Do not present the missing upright turn as measured.
- A follow-up three-minute bounded recording (`upright-35-0/1/2.raw`) contained static data only, with tilt about 11 degrees. Hence no new valid upright rotation comparison was obtained. One raw-stream interval skips 20 control/read indices (0.42 seconds); device read counters advance and failed-read counts do not increase. This is a recording gap, not evidence of 20 bus packet failures. Other recorded intervals are continuous. All decoded records remain model-off and torque-off.
- Restoration job `microduck-imu-diag-restore-1793735259` completed successfully. `restore-35-verified.json` confirms build `4734cbb-feetech-imu-auto-control.44`, original ExecStart, removed diagnostic override, active robotd/padd/duck-echo, IMU ready, all 15 joints fresh, model off and torque confirmed off. Original executable, calibration, execution profile and robot configuration hashes are unchanged. Reading the root process's `/proc/PID/exe` hash was denied to radxa; the verification uses service ExecStart, reported build and the original on-disk executable hash, and does not claim a new direct process hash.
- Head 0.5 and legs 0.7 remain unchanged. The experiment and raw logger are local diagnostic work, not a published walking fix. Before another upright test, synchronize operator readiness with the recording window; do not ask for walking or repeated falls to substitute for this missing passive measurement.

## Completed upright comparison (record 36)

After the operator explicitly prepared again, the temporary diagnostic runtime was re-enabled while unloaded and a new capture started before giving the turn instruction. The operator confirmed a completed upright turn and then put the robot down. `upright-36-0.raw` (SHA-256 7533c53bed9a1f04d1dcd5234d69ef465f36ddef7db39b99a82a0bd2ec71e115) contains 3,000 consecutive samples over 60 seconds, with zero new failed reads, no skipped read/control indices and model/torque off throughout. Raw-byte decoding still reproduces published state to floating-point roundoff.

At relative 22.079686 s, successive samples separated by 19.49 ms change raw gravity by 4.563169 degrees. The median output changes by 3.451206 degrees one sample later. The raw gyro reports only 0.748727 degrees of total rotation during the first interval; its gravity prediction residual is 4.579704 degrees. The source qxyz signs flip together while the reconstructed nonnegative w remains about 0.034 then 0.030. This is the specific three-component reconstruction boundary expected to amplify binary16 error. It is an observed attitude-path anomaly, not proof of every physical gait failure, and this recording does not contain independently measured true q4.

Reproduction: run parent `artifacts/bus-duck-check-20261104/imu-replay-32/inspect_quaternion_boundary.py` against `upright-36-0.raw`; the `.boundary.json` retains both original byte blocks and sequence/read indices. Firmware source forwards FIFO xyz unchanged. ST employee explanation confirms FIFO quantization and provides a full four-component ENDOP readout alternative: https://community.st.com/mems-sensors-48/lsm6dsv-half-precision-error-at-180-degrees-126519/index2.html . Exact flashed MCU build is not established by a version string alone.

Restoration job `microduck-imu-diag-restore-1793735569` completed successfully. `restore-36-verified.json` confirms original build4734cbb-feetech-imu-auto-control.44, removed override, restored robotd/padd/duck-echo, IMU ready/all15fresh, model off/confirmed torque off. No gait parameters changed. User subsequently asked for local candidate firmware and matching host work overnight and is powering the robot off; further work is offline only.
