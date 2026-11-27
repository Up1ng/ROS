"""Convert the simulator's ROS 8-bit RGB image, including row padding."""

import cv2
import numpy as np


FORMATS = {
    'rgb8': (3, cv2.COLOR_RGB2BGR),
    'bgr8': (3, None),
    'rgba8': (4, cv2.COLOR_RGBA2BGR),
    'bgra8': (4, cv2.COLOR_BGRA2BGR),
}


def to_bgr(message):
    try:
        channels, conversion = FORMATS[message.encoding.lower()]
    except KeyError as exc:
        raise ValueError(f'unsupported color encoding {message.encoding}') from exc
    row_width = message.width * channels
    if (message.height < 1 or row_width < channels or message.step < row_width
            or len(message.data) != message.height * message.step):
        raise ValueError('invalid image dimensions or stride')
    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
    image = rows[:, :row_width].reshape(message.height, message.width, channels)
    return cv2.cvtColor(image, conversion) if conversion is not None else image


def from_bgr(image, header=None):
    from sensor_msgs.msg import Image

    frame = Image()
    frame.height, frame.width = image.shape[:2]
    frame.encoding, frame.step = 'bgr8', frame.width * 3
    frame.data = image.tobytes()
    if header is not None:
        frame.header = header
    return frame
