# Summarize mc_test.py results, and compare a master/listener pair frame by frame.
#   python mc_analyze.py result.json [other.json]
import json
import sys

import numpy as np


def summarize(r):
    f = np.array([(a, b, c, d) for (a, b, c, d) in r['frames']], dtype=np.int64)
    if len(f) == 0:
        print('%s: no frames' % r['role']); return None
    ids, cam, host, inc = f[:, 0], f[:, 1], f[:, 2], f[:, 3].astype(bool)
    span = ids.max() - ids.min() + 1
    camSec = (cam.max() - cam.min())*1e-9
    dt = np.diff(cam)*1e-6
    gaps = np.diff(ids)
    print('%-9s access=%-9s packet=%-5s frames=%d (ids %d..%d, %d expected) incomplete=%d missing=%d timeouts=%d' %
          (r['role'], r['access'], r.get('packet_size'), len(ids), ids.min(), ids.max(), span, inc.sum(), span-len(np.unique(ids)), r['n_timeouts']))
    print('          rate from camera timestamps: %.3f fps; frame interval median %.4f ms, max %.3f ms; largest id gap %d' %
          ((len(ids)-1)/camSec if camSec > 0 else 0, np.median(dt), dt.max(), gaps.max() if len(gaps) else 0))
    if r.get('copy_ms_mean') is not None:
        print('          numpy copy per frame: %.3f ms' % r['copy_ms_mean'])
    return {int(i): (int(c), int(h), bool(x)) for (i, c, h, x) in zip(ids, cam, host, inc)}


rs = [json.load(open(p)) for p in sys.argv[1:]]
maps = [summarize(r) for r in rs]
if len(rs) == 2 and all(maps):
    (a, b) = maps
    common = sorted(set(a) & set(b))
    lo, hi = max(min(a), min(b)), min(max(a), max(b))  # Overlapping window of frame ids.
    inA = [i for i in a if lo <= i <= hi]
    inB = [i for i in b if lo <= i <= hi]
    print('\noverlap: frame ids %d..%d (%d frames); %s got %d, %s got %d, both got %d' % (lo, hi, hi-lo+1, rs[0]['role'], len(inA), rs[1]['role'], len(inB), len(common)))
    sameStamp = all(a[i][0] == b[i][0] for i in common)
    print('camera timestamps identical for every shared frame: %s' % sameStamp)
    d = np.array([(b[i][1] - a[i][1])*1e-6 for i in common])
    print('arrival time %s minus %s: median %.3f ms, 95th pct %.3f ms, min %.3f, max %.3f' %
          (rs[1]['role'], rs[0]['role'], np.median(d), np.percentile(d, 95), d.min(), d.max()))
