#!/usr/bin/env python
# Tests for standalone Kinefly: the .ufmf reader, the CSV sink, the realtime
# player, and an end-to-end headless run.  Run with:
#
#   pixi run test
#
import csv
import os
import shutil
import struct
import sys
import tempfile
import time
import unittest

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'nodes'))
import io_adapters
import kinefly
from kinefly_msgs import MsgFlystate


# write_ufmf()
# Write a ufmf v4 file the way BIAS does (bias/src/gui/video_writer_ufmf.cpp).
#   keyframes:  list of (stamp, image)
#   frames:     list of (stamp, [(col, row, box image), ...]); each frame uses the keyframe written before it.
#   order:      sequence of ('k', i) and ('f', i), the order to write the chunks.
#
def write_ufmf(filename, keyframes, frames, order, bIndex=True):
    (h, w) = keyframes[0][1].shape
    with open(filename, 'wb') as f:
        f.write(b'ufmf')
        f.write(struct.pack('<I', 4))
        locIndexPtr = f.tell()
        f.write(struct.pack('<Q', 0))
        f.write(struct.pack('<HHBB', w, h, 0, 5))
        f.write(b'MONO8')

        frameLocs, frameStamps, keyLocs, keyStamps = [], [], [], []
        for (kind, i) in order:
            if (kind == 'k'):
                (stamp, img) = keyframes[i]
                keyLocs.append(f.tell())
                keyStamps.append(stamp)
                f.write(struct.pack('<BB', 0, 4) + b'mean' + b'B')
                f.write(struct.pack('<HHd', img.shape[1], img.shape[0], stamp))
                f.write(np.ascontiguousarray(img, dtype=np.uint8).tobytes())
            else:
                (stamp, boxes) = frames[i]
                frameLocs.append(f.tell())
                frameStamps.append(stamp)
                f.write(struct.pack('<BdI', 1, stamp, len(boxes)))
                for (col, row, box) in boxes:
                    f.write(struct.pack('<HHHH', col, row, box.shape[1], box.shape[0]))
                    f.write(np.ascontiguousarray(box, dtype=np.uint8).tobytes())

        if (bIndex):
            def key(k):
                return struct.pack('<H', len(k)) + k
            def array(dtype, values, fmt):
                data = b''.join(struct.pack('<'+fmt, v) for v in values)
                return b'a' + dtype + struct.pack('<I', len(data)) + data
            f.write(struct.pack('<B', 2))
            locIndex = f.tell()
            f.write(b'd' + struct.pack('<B', 2))
            f.write(key(b'frame') + b'd' + struct.pack('<B', 2))
            f.write(key(b'loc') + array(b'q', frameLocs, 'q'))
            f.write(key(b'timestamp') + array(b'd', frameStamps, 'd'))
            f.write(key(b'keyframe') + b'd' + struct.pack('<B', 1))
            f.write(key(b'mean') + b'd' + struct.pack('<B', 2))
            f.write(key(b'loc') + array(b'q', keyLocs, 'q'))
            f.write(key(b'timestamp') + array(b'd', keyStamps, 'd'))
            f.seek(locIndexPtr)
            f.write(struct.pack('<Q', locIndex))


def make_movie(n_frames=40, fps=200.0, shape=(60, 80)):
    rng = np.random.default_rng(0)
    bg0 = rng.integers(0, 100, size=shape, dtype=np.uint8)
    bg1 = rng.integers(100, 200, size=shape, dtype=np.uint8)
    keyframes = [(0.0, bg0), (n_frames/2/fps, bg1)]
    frames = []
    expected = []
    for i in range(n_frames):
        bg = bg0 if (i < n_frames//2) else bg1
        img = bg.copy()
        boxes = []
        for (col, row, bw, bh) in [(i % 50, 5, 10, 8), (70, 50, 10, 10)]: # The second box hangs off the image's corner edge.
            box = np.full((bh, bw), (i*7) % 256, dtype=np.uint8)
            boxes.append((col, row, box))
            hClip = min(bh, shape[0]-row)
            wClip = min(bw, shape[1]-col)
            img[row:row+hClip, col:col+wClip] = box[:hClip, :wClip]
        frames.append((1000.0 + i/fps, boxes)) # BIAS stamps need not start at 0.
        expected.append(img)
    order = [('k', 0)] + [('f', i) for i in range(n_frames//2)] + [('k', 1)] + [('f', i) for i in range(n_frames//2, n_frames)]
    return (keyframes, frames, order, expected)


class TestUfmf(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp()
        (self.keyframes, self.frames, self.order, self.expected) = make_movie()

    def tearDown(self):
        shutil.rmtree(self.dir, ignore_errors=True)

    def check(self, filename, use_index):
        u = io_adapters.UfmfFile(filename, use_index=use_index)
        self.assertEqual(u.n_frames, len(self.frames))
        self.assertEqual(u.shape, self.expected[0].shape)
        for i in range(u.n_frames):
            (img, stamp) = u.read_frame(i)
            self.assertEqual(img.dtype, np.uint8)
            np.testing.assert_array_equal(img, self.expected[i], err_msg='frame %d' % i)
            self.assertAlmostEqual(stamp, self.frames[i][0])
        u.close()

    def test_with_index(self):
        filename = os.path.join(self.dir, 'a.ufmf')
        write_ufmf(filename, self.keyframes, self.frames, self.order, bIndex=True)
        self.check(filename, use_index=True)

    def test_scan_without_index(self):
        filename = os.path.join(self.dir, 'b.ufmf')
        write_ufmf(filename, self.keyframes, self.frames, self.order, bIndex=False)
        self.check(filename, use_index=True)  # Falls back to scanning.
        self.check(filename, use_index=False)

    def test_truncated_recording(self):
        filename = os.path.join(self.dir, 'c.ufmf')
        write_ufmf(filename, self.keyframes, self.frames, self.order, bIndex=False)
        size = os.path.getsize(filename)
        with open(filename, 'r+b') as f:
            f.truncate(size - 50) # Cut into the last frame.
        u = io_adapters.UfmfFile(filename)
        self.assertEqual(u.n_frames, len(self.frames)-1)
        u.close()

    def test_source_stamps_start_at_zero(self):
        filename = os.path.join(self.dir, 'd.ufmf')
        write_ufmf(filename, self.keyframes, self.frames, self.order)
        source = io_adapters.open_source(filename)
        self.assertIsInstance(source, io_adapters.UfmfSource)
        self.assertTrue(source.default_realtime)
        self.assertAlmostEqual(source.fps, 200.0, places=3)
        frame = source.read()
        self.assertEqual(frame.header.seq, 0)
        self.assertAlmostEqual(frame.header.stamp, 0.0)
        frame = source.read()
        self.assertAlmostEqual(frame.header.stamp, 1/200.0)
        source.close()

    def test_rejects_other_files(self):
        filename = os.path.join(self.dir, 'e.ufmf')
        with open(filename, 'wb') as f:
            f.write(b'not a ufmf file at all')
        with self.assertRaises(io_adapters.UfmfError):
            io_adapters.UfmfFile(filename)


def write_video(filename, frames, fps, fourcc):
    writer = cv2.VideoWriter(filename, cv2.VideoWriter_fourcc(*fourcc), fps, (frames[0].shape[1], frames[0].shape[0]), isColor=True)
    if (not writer.isOpened()):
        raise unittest.SkipTest('OpenCV cannot write %s with codec %s here.' % (filename, fourcc))
    for img in frames:
        writer.write(cv2.cvtColor(img, cv2.COLOR_GRAY2BGR))
    writer.release()


class TestVideoFileSource(unittest.TestCase):
    N = 30

    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.frames = []
        for i in range(self.N):
            img = np.full((64, 96), 30, np.uint8)
            cv2.circle(img, (10 + 2*i, 32), 6, 220, -1)
            self.frames.append(img)
        self.mp4 = os.path.join(self.dir, 'test.mp4')
        write_video(self.mp4, self.frames, 50.0, 'mp4v')

    def tearDown(self):
        shutil.rmtree(self.dir, ignore_errors=True)

    def read_all(self, source):
        frames = []
        while (True):
            frame = source.read()
            if (frame is None):
                break
            frames.append(frame)
        return frames

    def test_mp4_reads_every_frame_with_times(self):
        source = io_adapters.open_source(self.mp4)
        self.assertIsInstance(source, io_adapters.VideoFileSource)
        self.assertFalse(source.default_realtime)
        frames = self.read_all(source)
        self.assertEqual([f.header.seq for f in frames], list(range(self.N)))
        self.assertEqual(frames[0].image.shape, (64, 96))
        self.assertEqual(frames[0].image.dtype, np.uint8)
        np.testing.assert_allclose([f.header.stamp for f in frames], np.arange(self.N)/50.0, atol=1e-6)
        self.assertEqual(source.n_frames, self.N)
        source.close()

    def test_wrong_frame_count_in_header(self):
        for nHeader in [0, 5, 100]: # Missing, undercounted, overcounted.
            source = io_adapters.open_source(self.mp4)
            (source.n_frames, source.nFramesHeader) = (nHeader, nHeader)
            frames = self.read_all(source)
            self.assertEqual(len(frames), self.N, 'header count %d' % nHeader)
            self.assertEqual(source.n_frames, self.N)
            self.assertIsNone(source.read()) # Stays at the end.
            source.close()

    def test_rewind(self):
        source = io_adapters.open_source(self.mp4)
        first = source.read()
        for i in range(10):
            source.read()
        source.rewind()
        again = source.read()
        self.assertEqual(again.header.seq, 0)
        self.assertAlmostEqual(again.header.stamp, 0.0)
        np.testing.assert_array_equal(again.image, first.image)
        self.assertEqual(len(self.read_all(source)), self.N-1)
        source.close()


class TestRealtimePlayer(unittest.TestCase):
    def test_paces_frames_to_stamps(self):
        class Source(io_adapters.FrameSource):
            def __init__(self):
                io_adapters.FrameSource.__init__(self, 'test')
                self.n_frames = 50
            def _read_frame(self, i):
                return io_adapters.Frame(np.zeros((4, 4), np.uint8), seq=i, stamp=i*0.004)

        received = []
        player = io_adapters.RealtimePlayer(Source(), lambda frame: received.append((frame.header.seq, time.perf_counter())))
        player.start()
        player.bFinished.wait(5.0)
        self.assertEqual([seq for (seq, t) in received], list(range(50)))
        elapsed = received[-1][1] - received[0][1]
        self.assertGreater(elapsed, 49*0.004 - 0.002)  # Not faster than recorded...
        self.assertLess(elapsed, 49*0.004 + 0.1)       # ...and not much slower.


class TestCsvSink(unittest.TestCase):
    def test_rows(self):
        d = tempfile.mkdtemp()
        try:
            filename = os.path.join(d, 'out.csv')
            sink = io_adapters.CsvSink(filename, n_edges_max=2)
            state = MsgFlystate()
            state.head.angles = [0.25]
            state.head.radii = [50.0]
            state.left.angles = [0.5, 0.6, 0.7] # More than n_edges_max; extras dropped.
            state.aux.intensity = 0.75
            meta = {'frame': 7, 'stamp': 0.035, 't_acquired': 1.0, 't_start': 1.001, 't_done': 1.003, 'dropped_total': 2, 'scale': 0.5}
            sink.write(state, meta)
            state.head.angles = [9.0] # Changing the state after write() must not change the row.
            sink.close()
            with open(filename, newline='') as f:
                rows = list(csv.DictReader(f))
            self.assertEqual(len(rows), 1)
            r = rows[0]
            self.assertEqual(r['frame'], '7')
            self.assertAlmostEqual(float(r['latency_ms']), 3.0, places=3)
            self.assertAlmostEqual(float(r['process_ms']), 2.0, places=3)
            self.assertAlmostEqual(float(r['head_angle']), 0.25)
            self.assertAlmostEqual(float(r['head_radius']), 100.0) # Reported in unscaled pixels.
            self.assertAlmostEqual(float(r['left_angle_0']), 0.5)
            self.assertAlmostEqual(float(r['left_angle_1']), 0.6)
            self.assertEqual(r['right_angle_0'], '')
            self.assertEqual(r['abdomen_angle'], '')
            self.assertAlmostEqual(float(r['aux_intensity']), 0.75)
        finally:
            shutil.rmtree(d, ignore_errors=True)


class TestEndToEnd(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.dir, ignore_errors=True)

    def synthetic_fly(self, n, shape=(480, 640)):
        # A bright "wing" wedge sweeping around the right hinge on a dark background.
        frames = []
        for i in range(n):
            img = np.full(shape, 20, np.uint8)
            angle = np.deg2rad(20 + 40*np.sin(2*np.pi*i/25.0))
            p0 = (350, 200)
            p1 = (int(350 + 90*np.cos(angle)), int(200 + 90*np.sin(angle)))
            cv2.line(img, p0, p1, 230, 9)
            cv2.ellipse(img, (300, 200), (25, 60), 90, 0, 360, 180, -1) # The body.
            frames.append(img)
        return frames

    # run_kinefly()
    # mode:  'offline', 'realtime', or None for the file type's default.
    #
    def run_kinefly(self, video, mode, n_expected):
        csvfile = os.path.join(self.dir, 'out_%s.csv' % mode)
        self.last_csv = csvfile
        argv = [video, '--no-gui', '--gui-yaml', os.path.join(self.dir, 'gui.yaml'), '--csv', csvfile]
        if (mode is not None):
            argv.append('--%s' % mode)
        kinefly.main(argv)
        with open(csvfile, newline='') as f:
            rows = list(csv.DictReader(f))
        self.assertTrue(os.path.exists(os.path.join(self.dir, 'gui.yaml')))
        return rows

    def test_avi_offline_tracks_every_frame(self):
        video = os.path.join(self.dir, 'fly.avi')
        frames = self.synthetic_fly(60)
        writer = cv2.VideoWriter(video, cv2.VideoWriter_fourcc(*'MJPG'), 100.0, (640, 480), isColor=True)
        for img in frames:
            writer.write(cv2.cvtColor(img, cv2.COLOR_GRAY2BGR))
        writer.release()

        rows = self.run_kinefly(video, 'offline', 60)
        self.assertEqual([int(r['frame']) for r in rows], list(range(60)))
        self.assertAlmostEqual(float(rows[1]['stamp']), 0.01, places=6)
        right = np.array([float(r['right_angle']) for r in rows if r['right_angle'] != ''])
        self.assertGreater(len(right), 50)
        self.assertGreater(np.std(right), 0.05) # The sweeping wing is seen moving.

    def test_mp4_offline_tracks_every_frame(self):
        video = os.path.join(self.dir, 'fly.mp4')
        write_video(video, self.synthetic_fly(60), 100.0, 'mp4v')
        rows = self.run_kinefly(video, None, 60) # .mp4 defaults to offline.
        self.assertEqual([int(r['frame']) for r in rows], list(range(60)))
        self.assertAlmostEqual(float(rows[1]['stamp']), 0.01, places=6)
        self.assertTrue(os.path.basename(self.last_csv).startswith('out_'))
        right = np.array([float(r['right_angle']) for r in rows if r['right_angle'] != ''])
        self.assertGreater(np.std(right), 0.05)

    def test_ufmf_realtime(self):
        video = os.path.join(self.dir, 'fly.ufmf')
        frames = self.synthetic_fly(60)
        bg = np.full(frames[0].shape, 20, np.uint8)
        boxes = [(0.0 + i/200.0, [(250, 100, img[100:300, 250:450])]) for (i, img) in enumerate(frames)]
        write_ufmf(video, [(0.0, bg)], boxes, [('k', 0)] + [('f', i) for i in range(60)])

        rows = self.run_kinefly(video, 'realtime', 60)
        seqs = [int(r['frame']) for r in rows]
        self.assertGreater(len(seqs), 0)
        self.assertEqual(seqs, sorted(seqs))
        self.assertTrue(all(0 <= s < 60 for s in seqs))
        rows = self.run_kinefly(video, 'offline', 60)
        self.assertEqual([int(r['frame']) for r in rows], list(range(60)))


if __name__ == '__main__':
    unittest.main()
