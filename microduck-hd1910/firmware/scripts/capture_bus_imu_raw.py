#!/usr/bin/env python3
"""Bounded passive r17 capture; never opens UART or sends an actuator command.

The .raw file retains the original u32-LE framed stream, including calibration
metadata and same-cycle bus/state records. Decode with console/raw_telemetry.py.
Requires robotd's native.imu_raw diagnostic; an older daemon fails preflight.
"""
import argparse
import hashlib
import json
from pathlib import Path
import socket
import struct
import time

MAX_FRAME = 8 * 1024 * 1024
SOCKET = '/run/robotd.sock'


def exact(stream, size):
    result = bytearray()
    while len(result) < size:
        part = stream.read(size-len(result))
        if not part:
            raise EOFError('raw stream ended mid-record')
        result.extend(part)
    return bytes(result)


def read_frame(stream):
    header = exact(stream, 4)
    length = struct.unpack('<I', header)[0]
    if not 1 <= length <= MAX_FRAME:
        raise ValueError('raw frame outside bounded size')
    payload = exact(stream, length)
    if payload[0] not in (1, 2, 4):
        raise ValueError('unknown r17 record kind')
    return header + payload


def validate_bus(bus):
    native = bus.get('native', {})
    raw = native.get('imu_raw') or {}
    block = raw.get('block')
    if native.get('imu_source') != 'bus' or not isinstance(block, list) or len(block) != 12:
        raise ValueError('native.imu_raw unavailable; install diagnostic robotd before capture')
    if any(type(value) is not int or not 0 <= value <= 255 for value in block):
        raise ValueError('invalid imu_raw bytes')


def response(stream):
    line = stream.readline(1024*1024)
    if not line.endswith(b'\n'):
        raise ValueError('truncated or oversized RPC response')
    value = json.loads(line)
    if value.get('id') != 1 or value.get('error'):
        raise ValueError('read-only RPC rejected: ' + str(value.get('error')))
    return value['result']


def bus_status():
    with socket.socket(socket.AF_UNIX) as sock:
        sock.settimeout(3)
        sock.connect(SOCKET)
        sock.sendall(b'{"jsonrpc":"2.0","id":1,"method":"robot.busStatus","params":{}}\n')
        with sock.makefile('rb') as stream:
            return response(stream)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--expect-host', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=30)
    parser.add_argument('--max-mib', type=int, default=16)
    args = parser.parse_args()
    if socket.gethostname() != args.expect_host:
        raise ValueError('wrong robot hostname')
    if not 0 < args.seconds <= 300 or not 1 <= args.max_mib <= 64:
        raise ValueError('duration/size outside diagnostic bounds')
    manifest_path = args.output.with_suffix(args.output.suffix+'.json')
    if args.output.exists() or manifest_path.exists():
        raise FileExistsError('capture paths already exist; choose a new output')
    before = bus_status()
    validate_bus(before)
    manifest = dict(hostname=socket.gethostname(), wall_time=time.time(),
                    boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                    before=before, seconds=args.seconds, limit_bytes=args.max_mib*1024*1024,
                    framing='u32le', schema='r17.raw.v1', records=0, dynamic_records=0,
                    bytes=0, complete=False, stop_reason='starting')
    calibration = Path('/etc/robot/feetech-ft5/hd1910-calibration.toml')
    if calibration.is_file():
        manifest['calibration_sha256'] = hashlib.sha256(calibration.read_bytes()).hexdigest()
    started = time.monotonic()
    digest = hashlib.sha256()
    try:
        with socket.socket(socket.AF_UNIX) as sock:
            sock.settimeout(3)
            sock.connect(SOCKET)
            sock.sendall(b'{"jsonrpc":"2.0","id":1,"method":"robot.rawSubscribe","params":{}}\n')
            with sock.makefile('rb') as stream:
                ack = response(stream)
                if ack.get('accepted') is not True or ack.get('schema') != 'r17.raw.v1' or ack.get('framing') != 'u32le':
                    raise ValueError('unexpected raw stream handshake')
                manifest['ack'] = ack
                with args.output.open('xb') as output:
                    while time.monotonic()-started < args.seconds:
                        sock.settimeout(min(3, max(.01, args.seconds-(time.monotonic()-started))))
                        try:
                            frame = read_frame(stream)
                        except TimeoutError:
                            if time.monotonic()-started >= args.seconds:
                                break
                            raise
                        if manifest['bytes']+len(frame) > manifest['limit_bytes']:
                            manifest['stop_reason'] = 'size_limit'
                            break
                        output.write(frame)
                        digest.update(frame)
                        manifest['records'] += 1
                        manifest['dynamic_records'] += int(frame[4] == 2)
                        manifest['bytes'] += len(frame)
                if not manifest['dynamic_records']:
                    raise ValueError('no dynamic sample received')
                manifest['complete'] = True
                if manifest['stop_reason'] == 'starting':
                    manifest['stop_reason'] = 'duration'
    except BaseException as exc:
        manifest['stop_reason'] = type(exc).__name__+': '+str(exc)
        raise
    finally:
        manifest['elapsed_s'] = time.monotonic()-started
        manifest['sha256'] = digest.hexdigest()
        with manifest_path.open('x', encoding='utf8') as output:
            json.dump(manifest, output, ensure_ascii=False, indent=2)
        print(json.dumps({k:manifest[k] for k in ('complete','stop_reason','records','dynamic_records','bytes','elapsed_s')}))


if __name__ == '__main__':
    main()
