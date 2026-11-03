import io
import struct
import json
from pathlib import Path
import socket
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

import capture_bus_imu_raw as capture


class Fragmented(io.BytesIO):
    def read(self, size=-1):
        return super().read(min(size, 2))


class CaptureTests(unittest.TestCase):
    def test_full_q4_capture_requires_actual_full_bytes_and_descriptor(self):
        raw = dict(mode='full_q4_v1', descriptor=[0x51,0x34,1,1,200,28,120,0],
                   block=[0]*28, sequence=3, timestamp_us=24_000,
                   gyro_age_ms=2, quat_age_ms=3, status=0,
                   read_index=4, host_read_start_us=30_000, transaction_us=5000)
        native = dict(imu_source='bus', imu_full_raw=raw)
        capture.validate_bus({'native': native})
        for key in ('block', 'descriptor', 'sequence', 'timestamp_us', 'read_index',
                    'host_read_start_us', 'transaction_us', 'gyro_age_ms', 'quat_age_ms'):
            changed = dict(raw)
            del changed[key]
            with self.assertRaises(ValueError, msg=key):
                capture.validate_bus({'native': dict(native, imu_full_raw=changed)})
        with self.assertRaises(ValueError):
            capture.validate_bus({'native': dict(native, imu_raw={'block':[0]*12})})

    @unittest.skipUnless(sys.platform == 'linux', 'robot uses Linux Unix sockets')
    def test_bounded_capture_uses_only_read_routes_and_keeps_exact_records(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)/'capture.raw'
            address = str(Path(directory)/'robot.sock')
            native = {'imu_source':'bus', 'imu_raw':{'block':[0]*12}}
            meta = b'\x01'+json.dumps({'schema':'r17.raw.v1','channels':{}}).encode()
            sample = b'\x02\x08\0\0\0\0'
            frames = b''.join(struct.pack('<I',len(p))+p for p in (meta,sample))
            calls = []
            with socket.socket(socket.AF_UNIX) as server:
                server.bind(address)
                server.listen(2)
                server.settimeout(2)
                def peer():
                    for reply in ({'native':native}, {'accepted':True,'schema':'r17.raw.v1','framing':'u32le'}):
                        conn,_ = server.accept()
                        with conn, conn.makefile('rb') as stream:
                            calls.append(json.loads(stream.readline())['method'])
                            conn.sendall(json.dumps({'id':1,'result':reply}).encode()+b'\n')
                            if calls[-1] == 'robot.rawSubscribe':
                                conn.sendall(frames)
                                time.sleep(.15)
                worker = threading.Thread(target=peer, daemon=True)
                worker.start()
                args = ['capture','--expect-host',socket.gethostname(),'--output',str(output),'--seconds','.05']
                with patch.object(capture,'SOCKET',address), patch.object(sys,'argv',args):
                    capture.main()
                worker.join(timeout=2)
                self.assertFalse(worker.is_alive())
            self.assertEqual(calls,['robot.busStatus','robot.rawSubscribe'])
            self.assertEqual(output.read_bytes(),frames)
            manifest = json.loads(output.with_suffix('.raw.json').read_text())
            self.assertTrue(manifest['complete'])
            self.assertEqual(manifest['dynamic_records'],1)
            self.assertEqual(manifest['stop_reason'],'duration')
            self.assertLess(manifest['elapsed_s'],1)

    def test_fragmented_frame_retains_all_wire_bytes(self):
        payload = b'\x02\x08\0\0\0\0'
        framed = struct.pack('<I', len(payload)) + payload
        self.assertEqual(capture.read_frame(Fragmented(framed)), framed)

    def test_truncation_and_oversized_frames_fail(self):
        for data in (b'\x01', struct.pack('<I', 9) + b'\x02', struct.pack('<I', capture.MAX_FRAME + 1)):
            with self.assertRaises((EOFError, ValueError)):
                capture.read_frame(Fragmented(data))

    def test_old_runtime_is_not_reported_as_raw_imu_capture(self):
        with self.assertRaisesRegex(ValueError, 'imu_raw'):
            capture.validate_bus({'native': {'imu_source': 'bus'}})
        native = {'imu_source': 'bus', 'imu_raw': {'block': [0]*12}}
        capture.validate_bus({'native': native})
        native['imu_raw']['block'][-1] = 256
        with self.assertRaises(ValueError):
            capture.validate_bus({'native': native})


if __name__ == '__main__':
    unittest.main()
