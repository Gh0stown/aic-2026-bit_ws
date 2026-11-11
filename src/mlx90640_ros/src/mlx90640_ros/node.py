"""Headless ROS1 adapter for the standalone ESP32 MLX90640 stream."""
import math

import numpy as np

from . import core
from .transport import ReplayTransport, SerialTransport


def optional_float(value):
    if value is None or value == "":
        return None
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("temperature display limits must be finite")
    return result


def sample_to_messages(sample, color_image, stamp, frame_id,
                       image_type=None, temperature_type=None):
    """Convert one complete sample, sharing exactly one stamp across all topics.

    Message classes are injectable; this function needs neither a ROS master nor
    cv_bridge. Temperatures are Celsius, little-endian IEEE float32.
    """
    if image_type is None or temperature_type is None:
        from sensor_msgs.msg import Image, Temperature
        image_type = image_type or Image
        temperature_type = temperature_type or Temperature
    temperatures = np.ascontiguousarray(sample.temps, dtype="<f4")
    if temperatures.shape != (24, 32):
        raise ValueError("temperature image must be 24 rows by 32 columns")
    color = np.ascontiguousarray(color_image, dtype=np.uint8)
    if color.ndim != 3 or color.shape[2] != 3:
        raise ValueError("thermal image must be an HxWx3 BGR array")

    def image(array, encoding):
        message = image_type()
        message.header.stamp = stamp
        message.header.frame_id = frame_id
        message.height, message.width = array.shape[:2]
        message.encoding = encoding
        message.is_bigendian = 0
        message.step = array.shape[1] * array.dtype.itemsize * (3 if array.ndim == 3 else 1)
        message.data = array.tobytes()
        return message

    ambient = temperature_type()
    ambient.header.stamp = stamp
    ambient.header.frame_id = frame_id
    ambient.temperature = float(sample.ambient)
    ambient.variance = 0.0
    return image(temperatures, "32FC1"), image(color, "bgr8"), ambient


def run(rospy):
    """Run against an injected rospy module (all topics are relative)."""
    from sensor_msgs.msg import Image, Temperature

    capture = str(rospy.get_param("~capture_file", "") or "").strip()
    port = str(rospy.get_param("~port", "") or "").strip()
    if not capture and not port:
        raise ValueError("set an explicit non-empty ~port, or set ~capture_file for replay")
    if capture and port:
        raise ValueError("~port and ~capture_file are mutually exclusive")
    baud = int(rospy.get_param("~baud", 460800))
    if baud <= 0:
        raise ValueError("~baud must be positive")
    scale = int(rospy.get_param("~scale", 12))
    if scale < 1:
        raise ValueError("~scale must be positive")
    tmin = optional_float(rospy.get_param("~tmin", None))
    tmax = optional_float(rospy.get_param("~tmax", None))
    if tmin is not None and tmax is not None and tmin >= tmax:
        raise ValueError("~tmin must be less than ~tmax")
    frame_id = str(rospy.get_param("~frame_id", "mlx90640_link")).strip()
    if not frame_id or frame_id.startswith("/"):
        raise ValueError("~frame_id must be non-empty and have no leading slash")
    library = rospy.get_param("~library_path", "") or None
    decoder = core.Decoder(
        library_path=library,
        emissivity=float(rospy.get_param("~emissivity", 0.95)),
        tr_shift=float(rospy.get_param("~tr_shift", 8)),
        bad_pixels=bool(rospy.get_param("~bad_pixels", False)))

    def log(level, message):
        getattr(rospy, "logwarn" if level == "warning" else "loginfo")(message)

    options = dict(stop=rospy.is_shutdown, log=log)
    if capture:
        transport = ReplayTransport(
            capture, decoder, replay_rate=rospy.get_param("~replay_rate", 2.0),
            loop=rospy.get_param("~loop", False), **options)
    else:
        transport = SerialTransport(port, decoder, baud=baud, **options)
    publishers = (
        rospy.Publisher("temperature_image", Image, queue_size=1, latch=False),
        rospy.Publisher("thermal_image", Image, queue_size=1, latch=False),
        rospy.Publisher("ambient_temperature", Temperature, queue_size=1, latch=False))
    rospy.on_shutdown(transport.close)
    samples = transport.samples()
    try:
        for sample in samples:
            if rospy.is_shutdown():
                break
            color = core.render_image(sample, scale=scale, tmin=tmin, tmax=tmax)
            messages = sample_to_messages(sample, color, rospy.Time.now(), frame_id,
                                          Image, Temperature)
            for publisher, message in zip(publishers, messages):
                publisher.publish(message)
        if capture and not rospy.is_shutdown():
            # Wall-clock grace period works even when /use_sim_time has no clock.
            transport._wait(0.5)
    finally:
        samples.close()
        transport.close()
    return 0


def main():
    import rospy
    rospy.init_node("mlx90640_node", anonymous=False)
    try:
        return run(rospy)
    except (KeyboardInterrupt, rospy.ROSInterruptException):
        return 0
    except Exception as exc:
        rospy.logerr("MLX90640: %s", exc)
        return 1
