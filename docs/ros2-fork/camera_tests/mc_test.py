# Multicast test for the Lucid Triton2 with arena_api.
#   python mc_test.py --role unicast|master|listener --seconds N --out result.json [--flag FILE]
#
# unicast:   one receiver, multicast off (baseline: can this PC take the full stream?).
# master:    controls the camera, multicast on; touches FLAG once streaming.
# listener:  waits for FLAG, then opens the camera read-only with multicast on.
#
# Every role copies each frame into a numpy array (as a tracker would) and logs,
# per frame:  camera frame ID, camera timestamp (ns), host arrival time
# (time.perf_counter_ns, comparable across processes on this PC), incomplete flag.
# The master restores the camera settings it changed.
import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import arena_env  # noqa: F401  (makes arena_api importable; see arena_env.py)

import numpy as np
from arena_api.system import system

p = argparse.ArgumentParser()
p.add_argument('--role', required=True, choices=['unicast', 'master', 'listener'])
p.add_argument('--seconds', type=float, default=20.0)
p.add_argument('--out', required=True)
p.add_argument('--flag', default=None)
p.add_argument('--buffers', type=int, default=300)
args = p.parse_args()


def log(msg):
    print('[%s] %s' % (args.role, msg), flush=True)


if args.role == 'listener':
    t0 = time.time()
    while not (args.flag and os.path.exists(args.flag)):
        if time.time() - t0 > 30:
            log('master never started streaming'); sys.exit(1)
        time.sleep(0.1)
    time.sleep(1.0)

infos = []
t0 = time.time()
while not infos and time.time() - t0 < 20:
    infos = system.device_infos
    if not infos:
        time.sleep(1)
if not infos:
    log('no camera found'); sys.exit(1)
dev = system.create_device(infos[:1])[0]
nm, tl, ts = dev.nodemap, dev.tl_device_nodemap, dev.tl_stream_nodemap
access = tl['DeviceAccessStatus'].value
log('DeviceAccessStatus = %s' % access)

restore = {}
if args.role in ['unicast', 'master']:
    if access != 'ReadWrite':
        log('expected ReadWrite access'); sys.exit(1)
    restore['GevSCPSPacketSize'] = nm['GevSCPSPacketSize'].value

ts['StreamMulticastEnable'].value = (args.role != 'unicast')
ts['StreamBufferHandlingMode'].value = 'OldestFirst'  # Keep every frame, so misses are real misses.
if args.role in ['unicast', 'master']:
    ts['StreamAutoNegotiatePacketSize'].value = (os.environ.get('MC_NEGOTIATE', '1') == '1')
    ts['StreamPacketResendEnable'].value = True

info = {'role': args.role, 'access': access,
        'pixel_format': nm['PixelFormat'].value, 'width': nm['Width'].value, 'height': nm['Height'].value,
        'frame_rate_setting': nm['AcquisitionFrameRate'].value,
        'exposure_us': nm['ExposureTime'].value}

frames = []  # (frame_id, timestamp_ns, arrival_ns, incomplete)
nTimeouts = 0
copyNs = []
dev.start_stream(args.buffers)
try:
    info['packet_size'] = nm['GevSCPSPacketSize'].value
    log('streaming %s %dx%d @ %.3f fps setting, packet %d' % (info['pixel_format'], info['width'], info['height'], info['frame_rate_setting'], info['packet_size']))
    if args.role == 'master' and args.flag:
        open(args.flag, 'w').close()
    tEnd = time.perf_counter() + args.seconds
    while time.perf_counter() < tEnd:
        try:
            buf = dev.get_buffer(timeout=1000)
        except Exception as e:  # Timeout.
            nTimeouts += 1
            continue
        tArrive = time.perf_counter_ns()
        incomplete = bool(buf.is_incomplete)
        if not incomplete:
            t = time.perf_counter_ns()
            img = np.ctypeslib.as_array(buf.pdata, shape=(buf.height, buf.width)).copy()
            copyNs.append(time.perf_counter_ns() - t)
        frames.append((int(buf.frame_id), int(buf.timestamp_ns), tArrive, incomplete))
        dev.requeue_buffer(buf)
finally:
    dev.stop_stream()
    for (k, v) in restore.items():
        try:
            nm[k].value = v
        except Exception as e:
            log('could not restore %s: %s' % (k, e))
    system.destroy_device()

info.update({'n_frames': len(frames), 'n_timeouts': nTimeouts,
             'copy_ms_mean': (1e-6*np.mean(copyNs)) if copyNs else None,
             'frames': frames})
with open(args.out, 'w') as f:
    json.dump(info, f)
log('received %d frames (%d incomplete), %d timeouts' % (len(frames), sum(1 for f in frames if f[3]), nTimeouts))
