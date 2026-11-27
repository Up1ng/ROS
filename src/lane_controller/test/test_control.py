from dataclasses import replace

import cv2
from lane_controller.detector import DetectorConfig, detect
from lane_controller.pid import PID, PidConfig
import numpy as np
import pytest


def frame(offset=0):
    image = np.full((300, 400, 3), 40, np.uint8)
    for x, color in [(100, (0, 220, 255)), (300, (250, 250, 250))]:
        cv2.line(image, (x + offset, 130), (x + offset, 299), color, 9)
    return image


@pytest.mark.parametrize('offset,sign', [(-48, 1), (48, -1)])
def test_pixel_error_and_steering_sign(offset, sign):
    road = detect(frame(offset), DetectorConfig())
    assert road.error * sign > 0.1
    assert PID(PidConfig()).update(road.error, 1.0) * sign > 0


def test_single_boundary_and_bright_background_are_not_lane():
    image = frame()
    image[:, 220:] = 255
    assert detect(image, DetectorConfig()).error is None
    assert detect(frame(), replace(DetectorConfig(), yellow_left=False)).error is None


def test_real_dt_and_reset():
    pid = PID(PidConfig(kp=0.0, ki=1.0, kd=0.0))
    assert pid.update(0.4, 1.0) == 0
    assert pid.update(0.4, 1.1) == pytest.approx(0.04)
    assert pid.update(0.4, 1.3) == pytest.approx(0.12)
    assert pid.update(0.4, 2.0) == 0


def test_derivative_and_integral_do_not_build_up_during_saturation():
    pid = PID(PidConfig(kp=2.0, ki=1.0, kd=0.0, max_angular=0.5))
    for index in range(10):
        assert pid.update(0.8, 1.0 + index * 0.1) == 0.5
    assert pid.integral == 0
    d = PID(PidConfig(kp=0.0, kd=0.2, derivative_tau=0.0))
    assert d.update(0, 1.0) == 0
    assert d.update(0.1, 1.05) == pytest.approx(0.4)
    assert d.update(0.2, 1.25) == pytest.approx(0.1)


def test_decode_rgb_with_row_padding():
    from types import SimpleNamespace

    from lane_controller.image_codec import to_bgr

    image = SimpleNamespace(encoding='rgb8', height=2, width=1, step=4,
                            data=bytes([10, 20, 30, 0, 40, 50, 60, 0]))
    assert to_bgr(image).tolist() == [[[30, 20, 10]], [[60, 50, 40]]]
    image.step = 2
    with pytest.raises(ValueError, match='stride'):
        to_bgr(image)
