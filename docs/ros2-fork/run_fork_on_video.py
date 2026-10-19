#!/usr/bin/env python
# run_fork_on_video.py
# Run the elhananby/kinefly rewrite's tracking core (or a fork of it) offline on a
# video, every frame, and write a CSV with the same columns as standalone Kinefly
# (nodes/kinefly.py), so the two can be compared with nodes/compare_runs.py.
#
# Uses the same handle file (--gui-yaml) and parameter file (--params, the
# standalone format, e.g. params_example.yaml) as the standalone version.
#
#   python run_fork_on_video.py --fork PATH_TO_FORK VIDEO --gui-yaml HANDLES.yaml --params PARAMS.yaml --csv OUT.csv
#
# Then:
#   python nodes/kinefly.py VIDEO --offline --no-gui --scale 1.0 --gui-yaml COPY_OF_HANDLES.yaml --params PARAMS.yaml --csv ours.csv
#   python nodes/compare_runs.py ours.csv OUT.csv
#
# Run the two in separate processes:  the standalone nodes/kinefly.py and the
# rewrite's kinefly package have the same name.
#
import argparse
import csv
import os
import sys
import tempfile
import time

import yaml

# The original Kinefly defaults (kinefly.py in the 2016 code, and standalone Kinefly).
DEFAULTS = {
    'n_edges_max': 1, 'rc_background': 1000.0, 'wingbeat_min': 180.0, 'wingbeat_max': 220.0,
    'head':    {'tracker': 'area', 'autozero': True, 'threshold': 0.0, 'feathering': 0.0,  'saturation_correction': False},
    'abdomen': {'tracker': 'tip',  'autozero': True, 'threshold': 0.0, 'feathering': 0.25, 'saturation_correction': False},
    'left':    {'tracker': 'edge', 'autozero': True, 'threshold': 0.0, 'feathering': 0.25, 'saturation_correction': False},
    'right':   {'tracker': 'edge', 'autozero': True, 'threshold': 0.0, 'feathering': 0.25, 'saturation_correction': False},
}
PARTS = ['head', 'abdomen', 'left', 'right']


def main():
    parser = argparse.ArgumentParser(description='Run the Kinefly rewrite (or a fork) offline on a video and write a standalone-format CSV.')
    parser.add_argument('video')
    parser.add_argument('--fork', required=True, help='Path to the fork checkout (the directory containing src/kinefly).')
    parser.add_argument('--gui-yaml', required=True, help='Handle positions, as saved by standalone Kinefly (e.g. ~/kinefly.yaml).  Not modified.')
    parser.add_argument('--params', default=None, help='Standalone-format parameter file (e.g. params_example.yaml).  Default: the original defaults.')
    parser.add_argument('--csv', required=True)
    args = parser.parse_args()

    sys.path.insert(0, os.path.join(os.path.abspath(args.fork), 'src'))
    from kinefly.__main__ import _build_fly_params
    from kinefly.camera.videofile import VideoFileSource
    from kinefly.config.loader import load_rig_config
    from kinefly.core.events import EventBus
    from kinefly.fly import Fly

    # Merge the standalone parameter file over the original defaults.
    p = {k: (dict(v) if isinstance(v, dict) else v) for (k, v) in DEFAULTS.items()}
    if (args.params is not None):
        with open(args.params) as f:
            user = yaml.safe_load(f) or {}
        for (k, v) in user.items():
            if (k in PARTS) and isinstance(v, dict):
                p[k].update(v)
            elif (k in p):
                p[k] = v

    # The equivalent rig config for the rewrite.
    rig = {'kinefly': {'version': 2},
           'camera': {'source': 'videofile', 'path': args.video},
           'tracking': {'n_edges_max': p['n_edges_max'], 'rc_background': p['rc_background'],
                        'aux': {'wingbeat_min': float(p['wingbeat_min']), 'wingbeat_max': float(p['wingbeat_max'])}},
           'gui': {'state_file': os.path.abspath(args.gui_yaml)}}
    for part in PARTS:
        rig['tracking'][part] = p[part]
    with tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False) as f:
        yaml.safe_dump(rig, f)
        rig_file = f.name
    config = load_rig_config(rig_file)
    os.remove(rig_file)

    # Standalone Kinefly saves the handles at the top level; the rewrite expects them under 'gui'.
    with open(args.gui_yaml) as f:
        gui_state = {'gui': yaml.safe_load(f)}

    # realtime=False:  every frame, as fast as possible.  (The rewrite's own CLI always paces video files.)
    camera = VideoFileSource(args.video, realtime=False)
    camera.open()
    params = _build_fly_params(config, gui_state, img_size=camera.resolution)

    states = []
    bus = EventBus()
    bus.register(states.append)
    fly = Fly(params=params, event_bus=bus)

    n = max(1, int(p['n_edges_max']))
    columns = ['frame', 'stamp', 'process_ms']
    for part in PARTS:
        k = n if (part in ['left', 'right']) else 1
        columns += ['%s_angle' % part] if (k == 1) else ['%s_angle_%d' % (part, i) for i in range(k)]
        if (part in ['head', 'abdomen']):
            columns.append('%s_radius' % part)
    columns += ['aux_intensity', 'aux_freq']

    rows = []
    t0 = time.perf_counter()
    while True:
        try:
            (frame, stamp) = camera.read()
        except StopIteration:
            break
        tStart = time.perf_counter()
        fly.update(frame, stamp)
        tDone = time.perf_counter()
        s = states[-1] # A live reference; copy what's needed before the next update().
        row = [len(rows), '%.6f' % stamp, '%.3f' % (1000*(tDone-tStart))]
        for part in PARTS:
            state = getattr(s, part)
            k = n if (part in ['left', 'right']) else 1
            angles = list(state.angles)[:k]
            row += ['%.6f' % a for a in angles] + ['']*(k-len(angles))
            if (part in ['head', 'abdomen']):
                row.append('%.4f' % state.radii[0] if len(state.radii) else '')
        row += ['%.6f' % s.aux.intensity, '%.3f' % s.aux.freq]
        rows.append(row)
    elapsed = time.perf_counter() - t0
    camera.close()

    with open(args.csv, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(columns)
        w.writerows(rows)
    print('%d frames in %.1fs (%.1f frames/s including decoding) -> %s' % (len(rows), elapsed, len(rows)/elapsed if elapsed > 0 else 0, args.csv))


if __name__ == '__main__':
    main()
