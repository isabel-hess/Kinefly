#!/usr/bin/env python
# compare_runs.py
# Compare two Kinefly result CSVs of the same video, typically an offline run
# (every frame; the reference) against a realtime run (frames dropped when
# tracking fell behind).  Reports how many frames the realtime run tracked, and
# how far its values are from the reference on the frames both tracked.
#
#   python compare_runs.py video_kinefly_offline.csv video_kinefly_realtime.csv
#
import argparse
import csv
import sys

import numpy as np


def load(filename):
    with open(filename, newline='') as f:
        rows = list(csv.DictReader(f))
    if (len(rows) == 0):
        sys.exit('%s has no rows.' % filename)
    columns = list(rows[0].keys())
    data = {}
    for c in columns:
        data[c] = np.array([float(r[c]) if (r[c] != '') else np.nan for r in rows])
    return (columns, data)


def main(argv=None):
    parser = argparse.ArgumentParser(description='Compare a Kinefly realtime run against an offline (reference) run of the same video.')
    parser.add_argument('reference', help='CSV from the offline run.')
    parser.add_argument('test', help='CSV from the realtime run.')
    args = parser.parse_args(argv)

    (columnsRef, ref) = load(args.reference)
    (columnsTest, test) = load(args.test)

    framesRef = ref['frame'].astype(int)
    framesTest = test['frame'].astype(int)
    (common, iRef, iTest) = np.intersect1d(framesRef, framesTest, return_indices=True)

    print('Reference: %d frames (%s)' % (len(framesRef), args.reference))
    print('Test:      %d frames (%s)' % (len(framesTest), args.test))
    print('The test run tracked %d of the reference\'s %d frames (%.1f%%).' % (len(common), len(framesRef), 100.0*len(common)/len(framesRef)))

    gaps = np.diff(np.sort(framesTest))
    if (len(gaps) > 0) and (gaps.max() > 1):
        nGaps = np.sum(gaps > 1)
        print('Gaps in the test run: %d, the longest %d frames (%.1f ms at the reference framerate).' %
              (nGaps, gaps.max()-1, 1000.0*(gaps.max()-1)*np.nanmedian(np.diff(ref['stamp']))))

    skip = set(['frame', 'stamp', 't_acquired', 't_done', 'latency_ms', 'process_ms', 'dropped_total'])
    print('')
    print('%-20s %10s %12s %12s %12s' % ('value', 'n', 'mean |diff|', 'max |diff|', 'ref std'))
    for c in columnsRef:
        if (c in skip) or (c not in test):
            continue
        a = ref[c][iRef]
        b = test[c][iTest]
        ok = np.isfinite(a) & np.isfinite(b)
        if (not np.any(ok)):
            print('%-20s %10d %12s %12s %12s' % (c, 0, '-', '-', '-'))
            continue
        d = np.abs(a[ok]-b[ok])
        print('%-20s %10d %12.5f %12.5f %12.5f' % (c, np.sum(ok), d.mean(), d.max(), np.nanstd(ref[c])))

    for c in ['latency_ms', 'process_ms']:
        if (c in test):
            v = test[c]
            print('%s (test run): mean %.2f, 95th percentile %.2f, max %.2f' % (c, np.mean(v), np.percentile(v, 95), np.max(v)))


if __name__ == '__main__':
    main()
