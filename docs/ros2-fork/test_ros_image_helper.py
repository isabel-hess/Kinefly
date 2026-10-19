# Test the mono8 sensor_msgs/Image <-> numpy helper against the real ROS 2 message
# class, including row padding, and time it at the camera's frame size.
import time

import numpy as np
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image


def image_to_numpy(msg: Image) -> np.ndarray:
    """mono8 sensor_msgs/Image -> (height, width) uint8 array, without copying."""
    if msg.encoding not in ('mono8', '8UC1'):
        raise ValueError('unsupported encoding: %s' % msg.encoding)
    a = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    return a[:, :msg.width]  # Drop any row padding.


def numpy_to_image(a: np.ndarray, stamp: Time, frame_id: str = 'camera') -> Image:
    """(height, width) uint8 array -> mono8 sensor_msgs/Image."""
    if (a.ndim != 2) or (a.dtype != np.uint8):
        raise ValueError('expected a 2D uint8 array, got %s %s' % (a.shape, a.dtype))
    msg = Image(height=a.shape[0], width=a.shape[1], encoding='mono8', is_bigendian=0, step=a.shape[1])
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.data = np.ascontiguousarray(a).tobytes()
    return msg


rng = np.random.default_rng(0)
img = rng.integers(0, 256, size=(1080, 1440), dtype=np.uint8)
stamp = Time(sec=12, nanosec=345)

# Round trip.
msg = numpy_to_image(img, stamp)
back = image_to_numpy(msg)
assert back.shape == (1080, 1440) and back.dtype == np.uint8
assert np.array_equal(back, img)
assert (msg.header.stamp.sec, msg.header.stamp.nanosec) == (12, 345)
print('round trip ok; type of msg.data:', type(msg.data).__name__)

# A non-contiguous input (e.g. a crop) still works.
crop = img[100:300, 200:500]
assert np.array_equal(image_to_numpy(numpy_to_image(crop, stamp)), crop)

# Row padding (step > width), as some camera drivers send.
padded = Image(height=4, width=5, encoding='mono8', is_bigendian=0, step=8)
rows = np.arange(32, dtype=np.uint8).reshape(4, 8)
padded.data = rows.tobytes()
assert np.array_equal(image_to_numpy(padded), rows[:, :5])
print('padding ok')

# Wrong encoding is rejected.
try:
    image_to_numpy(Image(height=1, width=1, encoding='bgr8', step=3, data=b'\0\0\0'))
    raise AssertionError('bgr8 was accepted')
except ValueError:
    print('non-mono8 rejected ok')

# Is it zero-copy?
base = image_to_numpy(msg)
print('image_to_numpy shares memory with msg.data:', not base.flags['OWNDATA'])

# Timing at 1440x1080 (the frame budget at 166.3 fps is 6.01 ms).
def timeit(f, n=200):
    t = time.perf_counter()
    for i in range(n):
        f()
    return 1000.0*(time.perf_counter() - t)/n
print('image_to_numpy: %.3f ms/frame' % timeit(lambda: image_to_numpy(msg)))
print('numpy_to_image: %.3f ms/frame' % timeit(lambda: numpy_to_image(img, stamp)))
