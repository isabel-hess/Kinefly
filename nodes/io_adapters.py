#!/usr/bin/env python
# io_adapters.py
# Frame sources (where images come from) and result sinks (where tracking
# results go) for standalone Kinefly.  The tracking core only ever sees a
# Frame; adding a live camera or a BIAS plugin means adding a FrameSource.
#
import csv
import mmap
import os
import queue
import struct
import threading
import time

import cv2
import numpy as np

import kinefly_runtime as runtime
from kinefly_msgs import Header


###############################################################################
###############################################################################
# A grayscale image plus its timing.
#   image:      2D uint8 array.
#   header:     Header(seq=frame index in the source, stamp=seconds from the source clock).
#   t_acquired: runtime.now() when the frame was handed to the tracker.
#
class Frame(object):
    def __init__(self, image, seq, stamp, t_acquired=None):
        self.image = image
        self.header = Header(seq=seq, stamp=stamp, frame_id='camera')
        self.t_acquired = runtime.now() if (t_acquired is None) else t_acquired


###############################################################################
###############################################################################
# Base class for sequential frame sources.  Subclasses implement _read_frame(i).
#
class FrameSource(object):
    default_realtime = False

    def __init__(self, path):
        self.path = path
        self.n_frames = 0
        self.fps = 0.0
        self.shape = None # (height, width)
        self.iFrame = 0   # Index of the next frame read() will return.

    # read()
    # Return the next Frame, or None at the end of the source.
    #
    def read(self):
        if (self.iFrame >= self.n_frames):
            return None
        frame = self._read_frame(self.iFrame)
        if (frame is not None):
            self.iFrame += 1
        return frame

    def rewind(self):
        self.iFrame = 0

    def close(self):
        pass

    def _read_frame(self, i):
        raise NotImplementedError


###############################################################################
###############################################################################
# Any video file OpenCV can decode (.avi, .mp4, .mov, ...).  Frames are
# converted to grayscale.
#
# Stamps come from the container's per-frame presentation times, so a variable
# framerate .mp4 is timed correctly; for constant-rate files they equal index/fps.
# If the container gives no usable times, stamps fall back to index/fps.
#
# The frame count in the file header is only an estimate (in .mp4 especially it
# can be off either way, or missing), so frames are read until decoding stops,
# and n_frames is corrected then.
#
class VideoFileSource(FrameSource):
    default_realtime = False

    def __init__(self, path):
        FrameSource.__init__(self, path)
        self._open()
        if (not self.capture.isOpened()):
            raise IOError('Could not open video file: %s' % path)

        self.fps = self.capture.get(cv2.CAP_PROP_FPS)
        if (not self.fps) or (self.fps <= 0) or (not np.isfinite(self.fps)):
            runtime.logwarn('%s: no framerate in the file; assuming 30 fps.' % path)
            self.fps = 30.0
        self.n_frames = max(0, int(self.capture.get(cv2.CAP_PROP_FRAME_COUNT)))
        self.nFramesHeader = self.n_frames
        self.bCountExact = False # Becomes True once we've read to the end.
        self.shape = (int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT)), int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH)))
        self.bUseContainerTimes = True
        self.msPrev = None
        self.ms0 = 0.0

    def _open(self):
        self.capture = cv2.VideoCapture(self.path)
        self.iCapture = 0 # Index of the frame the capture will decode next.

    # read()
    # Unlike the base class, don't stop at n_frames until it's known to be exact.
    #
    def read(self):
        if (self.bCountExact) and (self.iFrame >= self.n_frames):
            return None
        frame = self._read_frame(self.iFrame)
        if (frame is not None):
            self.iFrame += 1
        return frame

    def _read_frame(self, i):
        if (i != self.iCapture):
            if (i == 0):
                # Rewind by reopening; seeking is only approximate in some codecs (e.g. H.264).
                self.capture.release()
                self._open()
            else:
                self.capture.set(cv2.CAP_PROP_POS_FRAMES, i)
                self.iCapture = i
                self.msPrev = None # Don't compare times across the seek.

        (ok, img) = self.capture.read()
        if (not ok) or (img is None):
            if (not self.bCountExact) and (i != self.nFramesHeader):
                runtime.logwarn('%s: the file header says %d frames, but the video has %d.' % (self.path, self.nFramesHeader, i))
            self.n_frames = i
            self.bCountExact = True
            return None
        self.iCapture += 1
        if (i >= self.n_frames):
            self.n_frames = i+1 # The header undercounted.

        # The time of this frame from the container, in ms from the start of the video.
        stamp = i/self.fps
        if (self.bUseContainerTimes):
            ms = self.capture.get(cv2.CAP_PROP_POS_MSEC)
            if (i == 0):
                self.ms0 = ms
                self.msPrev = ms
            elif (self.msPrev is None) or (ms > self.msPrev):
                self.msPrev = ms
            else:
                runtime.logwarn('%s: no usable per-frame times in the file; using frame index / %.3f fps.' % (self.path, self.fps))
                self.bUseContainerTimes = False
            if (self.bUseContainerTimes):
                stamp = (ms - self.ms0)/1000.0

        if (img.ndim == 3):
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        return Frame(img, seq=i, stamp=stamp)

    def close(self):
        self.capture.release()


###############################################################################
###############################################################################
class UfmfError(Exception):
    pass


# UfmfFile
# Random-access reader for the micro fly movie format (.ufmf) version 4, as
# written by BIAS (see bias/src/gui/video_writer_ufmf.cpp).  All values are
# little-endian.
#
#   Header:   b'ufmf', uint32 version, uint64 index_loc,
#             uint16 max_width, uint16 max_height, uint8 is_fixed_size,
#             uint8 len, char coding[len]  (e.g. 'MONO8')
#   Chunks:   uint8 chunk_id, then
#     0 keyframe: uint8 len, char type[len] ('mean'), char dtype ('B'),
#                 uint16 width, uint16 height, float64 stamp, pixels[height*width]
#     1 frame:    float64 stamp, uint32 n_boxes, then per box:
#                 uint16 col, uint16 row, uint16 width, uint16 height, pixels[height*width]
#     2 index:    a nested dict (see _parse_index()) with
#                 frame.loc, frame.timestamp, keyframe.mean.loc, keyframe.mean.timestamp
#
# A frame is the most recent keyframe (the background image) with that frame's
# boxes (the foreground pixels) pasted on top.
#
class UfmfFile(object):
    KEYFRAME_CHUNK_ID = 0
    FRAME_CHUNK_ID = 1
    INDEX_CHUNK_ID = 2
    DTYPES = {b'B':'<u1', b'b':'<i1', b'H':'<u2', b'h':'<i2', b'I':'<u4', b'i':'<i4',
              b'Q':'<u8', b'q':'<i8', b'f':'<f4', b'd':'<f8'}

    def __init__(self, path, use_index=True):
        self.path = path
        self.file = open(path, 'rb')
        try:
            self.buf = mmap.mmap(self.file.fileno(), 0, access=mmap.ACCESS_READ)
        except ValueError: # Empty file.
            self.file.close()
            raise UfmfError('%s is empty.' % path)
        try:
            self._open(use_index)
        except Exception:
            self.close()
            raise


    def _open(self, use_index):
        path = self.path
        self._keyframeCached = None
        self._parse_header()

        index = None
        if (use_index) and (self.index_loc > 0):
            try:
                index = self._parse_index(self.index_loc)
            except (UfmfError, struct.error, KeyError, IndexError, ValueError) as e:
                runtime.logwarn('%s: unreadable index (%s); scanning the file instead.' % (path, e))
                index = None

        if (index is not None):
            self.frame_locs = np.asarray(index['frame']['loc'], dtype=np.int64)
            self.frame_stamps = np.asarray(index['frame']['timestamp'], dtype=np.float64)
            self.keyframe_locs = np.asarray(index['keyframe']['mean']['loc'], dtype=np.int64)
            self.keyframe_stamps = np.asarray(index['keyframe']['mean']['timestamp'], dtype=np.float64)
        else:
            self._scan()

        if (len(self.keyframe_locs)==0):
            raise UfmfError('%s has no background keyframe.' % path)

        # Keyframes are written before the frames that use them, so each frame
        # uses the last keyframe that precedes it in the file.
        order = np.argsort(self.keyframe_locs)
        self.keyframe_locs = self.keyframe_locs[order]
        self.keyframe_stamps = self.keyframe_stamps[order]
        self.iKeyframeOfFrame = np.maximum(np.searchsorted(self.keyframe_locs, self.frame_locs) - 1, 0)

        self.n_frames = len(self.frame_locs)
        self.shape = self.read_keyframe(0).shape
        self._iKeyframeCached = None
        self._keyframeCached = None


    def _parse_header(self):
        if (self.buf[0:4] != b'ufmf'):
            raise UfmfError('%s is not a ufmf file.' % self.path)
        (self.version, self.index_loc) = struct.unpack_from('<IQ', self.buf, 4)
        if (self.version != 4):
            raise UfmfError('%s: ufmf version %d is not supported (only version 4).' % (self.path, self.version))
        (self.max_width, self.max_height, self.is_fixed_size, len_coding) = struct.unpack_from('<HHBB', self.buf, 16)
        self.coding = self.buf[22:22+len_coding].decode('latin-1')
        if (self.coding != 'MONO8'):
            raise UfmfError('%s: coding %s is not supported (only MONO8).' % (self.path, self.coding))
        self.loc_first_chunk = 22 + len_coding


    # _parse_index()
    # The index is a nested dict:  'd', uint8 n_keys, then n_keys of (uint16 len, char key[len], value),
    # where each value is either another dict, or an array:  'a', char dtype, uint32 n_bytes, data[n_bytes].
    #
    def _parse_index(self, loc):
        if (self.buf[loc-1] != self.INDEX_CHUNK_ID):
            raise UfmfError('index location does not point at an index chunk')
        (value, loc) = self._parse_index_value(loc)
        return value

    def _parse_index_value(self, loc):
        tag = self.buf[loc:loc+1]
        loc += 1
        if (tag == b'd'):
            (n_keys,) = struct.unpack_from('<B', self.buf, loc)
            loc += 1
            d = {}
            for i in range(n_keys):
                (len_key,) = struct.unpack_from('<H', self.buf, loc)
                loc += 2
                key = self.buf[loc:loc+len_key].decode('latin-1')
                loc += len_key
                (d[key], loc) = self._parse_index_value(loc)
            return (d, loc)
        elif (tag == b'a'):
            dtype = np.dtype(self.DTYPES[self.buf[loc:loc+1]])
            (n_bytes,) = struct.unpack_from('<I', self.buf, loc+1)
            loc += 5
            a = np.frombuffer(self.buf, dtype=dtype, count=n_bytes//dtype.itemsize, offset=loc).copy()
            return (a, loc+n_bytes)
        else:
            raise UfmfError('unknown index tag %r' % tag)


    # _scan()
    # Build the frame and keyframe locations by walking the chunks, for files without a usable index.
    #
    def _scan(self):
        frame_locs, frame_stamps, keyframe_locs, keyframe_stamps = [], [], [], []
        loc = self.loc_first_chunk
        size = len(self.buf)
        try:
            while (loc < size):
                chunk_id = self.buf[loc]
                if (chunk_id == self.KEYFRAME_CHUNK_ID):
                    (stamp, loc_next) = self._keyframe_info(loc)
                    keyframe_locs.append(loc)
                    keyframe_stamps.append(stamp)
                elif (chunk_id == self.FRAME_CHUNK_ID):
                    (stamp, n_boxes) = struct.unpack_from('<dI', self.buf, loc+1)
                    loc_next = loc + 13
                    for iBox in range(n_boxes):
                        (col, row, w, h) = struct.unpack_from('<HHHH', self.buf, loc_next)
                        loc_next += 8 + w*h
                    if (loc_next > size):
                        break # Truncated final frame.
                    frame_locs.append(loc)
                    frame_stamps.append(stamp)
                else:
                    break # The index, or garbage from an interrupted recording.
                loc = loc_next
        except struct.error:
            pass # Truncated file.

        self.frame_locs = np.array(frame_locs, dtype=np.int64)
        self.frame_stamps = np.array(frame_stamps, dtype=np.float64)
        self.keyframe_locs = np.array(keyframe_locs, dtype=np.int64)
        self.keyframe_stamps = np.array(keyframe_stamps, dtype=np.float64)


    # _keyframe_info()
    # Returns (stamp, location after the chunk) and sets self._kf_* for the keyframe at loc.
    #
    def _keyframe_info(self, loc):
        if (self.buf[loc] != self.KEYFRAME_CHUNK_ID):
            raise UfmfError('no keyframe at location %d' % loc)
        len_type = self.buf[loc+1]
        p = loc + 2 + len_type
        dtype = self.buf[p:p+1]
        if (dtype != b'B'):
            raise UfmfError('keyframe dtype %r is not supported (only uint8)' % dtype)
        (w, h, stamp) = struct.unpack_from('<HHd', self.buf, p+1)
        self._kf_w, self._kf_h, self._kf_data = w, h, p+13
        return (stamp, p + 13 + w*h)


    def read_keyframe(self, iKeyframe):
        self._keyframe_info(int(self.keyframe_locs[iKeyframe]))
        # Copy, so no views into the memory map outlive close().
        return np.frombuffer(self.buf, dtype=np.uint8, count=self._kf_w*self._kf_h, offset=self._kf_data).reshape(self._kf_h, self._kf_w).copy()


    # read_frame()
    # Return (image, stamp) for frame i.  The image is a new writable uint8 array.
    #
    def read_frame(self, i):
        iKeyframe = int(self.iKeyframeOfFrame[i])
        if (iKeyframe != self._iKeyframeCached):
            self._keyframeCached = self.read_keyframe(iKeyframe)
            self._iKeyframeCached = iKeyframe
        img = self._keyframeCached.copy()
        (height, width) = img.shape

        loc = int(self.frame_locs[i])
        if (self.buf[loc] != self.FRAME_CHUNK_ID):
            raise UfmfError('no frame at location %d' % loc)
        (stamp, n_boxes) = struct.unpack_from('<dI', self.buf, loc+1)
        loc += 13
        buf = self.buf
        for iBox in range(n_boxes):
            (col, row, w, h) = struct.unpack_from('<HHHH', buf, loc)
            loc += 8
            if (w > 0) and (h > 0):
                box = np.frombuffer(buf, dtype=np.uint8, count=w*h, offset=loc).reshape(h, w)
                if (row+h > height) or (col+w > width): # Clip boxes that hang off the image.
                    box = box[:max(0, height-row), :max(0, width-col)]
                img[row:row+box.shape[0], col:col+box.shape[1]] = box
            loc += w*h
        box = None

        return (img, stamp)


    def close(self):
        self._keyframeCached = None
        try:
            self.buf.close()
        except BufferError:
            pass # A view into the map is still alive somewhere; it's released with the process.
        self.file.close()


class UfmfSource(FrameSource):
    default_realtime = True

    def __init__(self, path, use_index=True):
        FrameSource.__init__(self, path)
        self.ufmf = UfmfFile(path, use_index=use_index)
        self.n_frames = self.ufmf.n_frames
        self.shape = self.ufmf.shape
        stamps = self.ufmf.frame_stamps
        self.stamp0 = stamps[0] if (len(stamps)>0) else 0.0
        self.fps = (len(stamps)-1)/(stamps[-1]-stamps[0]) if (len(stamps)>1) and (stamps[-1]>stamps[0]) else 0.0

    def _read_frame(self, i):
        (img, stamp) = self.ufmf.read_frame(i)
        return Frame(img, seq=i, stamp=stamp-self.stamp0)

    def close(self):
        self.ufmf.close()


def open_source(path):
    ext = os.path.splitext(path)[1].lower()
    if (ext == '.ufmf'):
        return UfmfSource(path)
    else:
        return VideoFileSource(path)


###############################################################################
###############################################################################
# RealtimePlayer
# Replays a FrameSource on its own thread, handing each frame to callback() at
# the moment it would have arrived from the camera (per the frame stamps).  If
# the consumer falls behind, it is the consumer's job to drop frames; the player
# never waits for it.  This stands in for a live camera feed.
#
class RealtimePlayer(object):
    def __init__(self, source, callback, loop=False):
        self.source = source
        self.callback = callback
        self.loop = loop
        self.bPaused = threading.Event()
        self.bStop = threading.Event()
        self.bFinished = threading.Event()
        self.nEmitted = 0
        self.lagMax = 0.0 # Worst lateness of the player itself (seconds), e.g. from slow decoding.
        self.thread = threading.Thread(target=self._run, name='RealtimePlayer', daemon=True)

    def start(self, paused=False):
        if (paused):
            self.bPaused.set()
        self.thread.start()

    def pause(self):
        self.bPaused.set()

    def resume(self):
        self.bPaused.clear()

    def is_paused(self):
        return self.bPaused.is_set()

    def stop(self):
        self.bStop.set()
        self.thread.join(timeout=2.0)

    def _run(self):
        tWall0 = None
        stamp0 = None
        while (not self.bStop.is_set()):
            if (self.bPaused.is_set()):
                tWall0 = None # Re-base the clock on resume.
                time.sleep(0.01)
                continue

            frame = self.source.read()
            if (frame is None):
                if (self.loop) and (self.source.n_frames > 0):
                    self.source.rewind()
                    tWall0 = None
                    continue
                break

            if (tWall0 is None):
                tWall0 = runtime.now()
                stamp0 = frame.header.stamp

            # Wait until this frame's moment.  Sleep most of the way (Windows oversleeps by ~0.5ms), then
            # yield until due.  sleep(0) releases the GIL, so the wait doesn't slow the tracker down.
            tDue = tWall0 + (frame.header.stamp - stamp0)
            while (True):
                tRemaining = tDue - runtime.now()
                if (tRemaining <= 0.0) or (self.bStop.is_set()):
                    break
                time.sleep(tRemaining - 0.0008 if (tRemaining > 0.0008) else 0)
            self.lagMax = max(self.lagMax, runtime.now() - tDue)

            # The frame "arrived" when it was due, so any lateness of the player counts toward latency.
            frame.t_acquired = tDue
            self.nEmitted += 1
            self.callback(frame)

        self.bFinished.set()


###############################################################################
###############################################################################
# CsvSink
# Writes one row per tracked frame.  write() only formats the row and queues it
# (the flystate objects are live, so they must be copied immediately); a
# background thread does the file I/O so disk stalls never block tracking.
#
class CsvSink(object):
    PARTS = ['head', 'abdomen', 'left', 'right']

    def __init__(self, path, n_edges_max=1, flush_interval=1.0):
        self.path = path
        self.n_edges_max = max(1, int(n_edges_max))
        self.flush_interval = flush_interval
        self.queue = queue.Queue()
        self.nRows = 0

        columns = ['frame', 'stamp', 't_acquired', 't_done', 'latency_ms', 'process_ms', 'dropped_total']
        for part in self.PARTS:
            n = self.n_edges_max if (part in ['left', 'right']) else 1
            if (n == 1):
                columns.append('%s_angle' % part)
            else:
                columns.extend(['%s_angle_%d' % (part, i) for i in range(n)])
            if (part in ['head', 'abdomen']):
                columns.append('%s_radius' % part)
        columns.extend(['aux_intensity', 'aux_freq'])
        self.columns = columns

        self.file = open(path, 'w', newline='', buffering=1024*1024)
        self.writer = csv.writer(self.file)
        self.writer.writerow(columns)
        self.thread = threading.Thread(target=self._run, name='CsvSink', daemon=True)
        self.thread.start()

    # write()
    # flystate:     MsgFlystate from Fly.get_flystate().
    # meta:         dict with frame, stamp, t_acquired, t_start, t_done, dropped_total, scale.
    #
    def write(self, flystate, meta):
        scale = meta.get('scale', 1.0)
        row = [meta['frame'],
               '%.6f' % meta['stamp'],
               '%.6f' % meta['t_acquired'],
               '%.6f' % meta['t_done'],
               '%.3f' % (1000.0*(meta['t_done']-meta['t_acquired'])),
               '%.3f' % (1000.0*(meta['t_done']-meta['t_start'])),
               meta.get('dropped_total', 0)]
        for part in self.PARTS:
            state = getattr(flystate, part)
            n = self.n_edges_max if (part in ['left', 'right']) else 1
            angles = list(state.angles)[:n]
            row.extend(['%.6f' % a for a in angles] + ['']*(n-len(angles)))
            if (part in ['head', 'abdomen']):
                radii = list(state.radii)
                row.append('%.4f' % (radii[0]/scale) if (len(radii)>0) else '')
        row.append('%.6f' % flystate.aux.intensity)
        row.append('%.3f' % flystate.aux.freq)
        self.queue.put(row)
        self.nRows += 1

    def _run(self):
        tFlush = time.monotonic()
        while (True):
            try:
                row = self.queue.get(timeout=self.flush_interval)
            except queue.Empty:
                row = None
            if (row is StopIteration):
                break
            if (row is not None):
                self.writer.writerow(row)
            if (time.monotonic()-tFlush >= self.flush_interval):
                self.file.flush()
                tFlush = time.monotonic()
        self.file.flush()

    def close(self):
        self.queue.put(StopIteration)
        self.thread.join()
        self.file.close()
