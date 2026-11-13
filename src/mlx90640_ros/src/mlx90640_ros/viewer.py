"""Standalone viewer sharing exactly the ROS adapter's decoder and transport."""
import argparse
import math
from pathlib import Path
import sys
import time

from . import core
from .hotspot import find_hotspot
from .transport import ReplayTransport, SerialTransport, positive_rate


def build_parser():
    parser = argparse.ArgumentParser(description="ESP32 MLX90640 viewer (no ROS required)")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--port", help="explicit ESP32 serial port; never auto-detected")
    source.add_argument("--capture", help="recorded capture.bin, no hardware is opened")
    parser.add_argument("--lib", default=None, help="path to the MLX90640 shared library")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument("--emissivity", type=float, default=0.95)
    parser.add_argument("--tr-shift", type=float, default=8)
    parser.add_argument("--bad-pixels", action="store_true", help="enable bad-pixel correction")
    parser.add_argument("--no-mirror-x", dest="mirror_x", action="store_false",
                        default=True, help="disable the default IR horizontal mirror correction")
    parser.add_argument("--headless", action="store_true", help="do not open GUI windows")
    parser.add_argument("--save-dir", default="~/Pictures/mlx90640")
    parser.add_argument("--save-every", type=int, default=10,
                        help="save every N complete images; 0 disables automatic saving")
    parser.add_argument("--scale", type=int, default=12)
    parser.add_argument("--tmin", type=float, default=20.0,
                        help="fixed color scale minimum in Celsius (default: 20)")
    parser.add_argument("--tmax", type=float, default=35.0,
                        help="fixed color scale maximum in Celsius (default: 35)")
    parser.add_argument("--duration", type=float, help="stop after this many wall-clock seconds")
    parser.add_argument("--replay-rate", type=positive_rate, default=2.0,
                        help="capture subpages per second; default 2 => 1 full image/s")
    parser.add_argument("--loop", action="store_true", help="repeat capture until stopped")
    return parser


def save_image(image, directory, index, imwrite=None):
    """Save outside the repository; report codec or filesystem errors explicitly."""
    directory = Path(directory).expanduser().resolve()
    roots = set()
    for location in (Path(__file__).resolve(), Path.cwd()):
        for ancestor in (location,) + tuple(location.parents):
            if (ancestor / ".git").exists():
                roots.add(ancestor)
    for root in roots:
        if directory == root or root in directory.parents:
            raise ValueError("save directory must be outside the repository: %s" % directory)
    if imwrite is None:
        import cv2
        imwrite = cv2.imwrite
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / ("mlx90640_%d_%06d.png" % (time.time_ns(), index))
    if not imwrite(str(path), image):
        raise IOError("failed to save thermal image: %s" % path)
    return path


def run(args):
    if args.baud <= 0:
        raise ValueError("--baud must be positive")
    for limit in (args.tmin, args.tmax):
        if limit is not None and not math.isfinite(limit):
            raise ValueError("--tmin and --tmax must be finite")
    if args.scale < 1:
        raise ValueError("--scale must be positive")
    if args.save_every < 0:
        raise ValueError("--save-every cannot be negative")
    if args.duration is not None:
        if not math.isfinite(args.duration) or args.duration <= 0:
            raise ValueError("--duration must be finite and positive")
    if args.tmin is not None and args.tmax is not None and args.tmin >= args.tmax:
        raise ValueError("--tmin must be less than --tmax")
    decoder = core.Decoder(library_path=args.lib or None, emissivity=args.emissivity,
                           tr_shift=args.tr_shift, bad_pixels=args.bad_pixels,
                           mirror_x=args.mirror_x)
    cv2 = None
    if not args.headless:
        import cv2
    started = time.monotonic()
    latest = None
    count = 0
    quit_requested = False
    window_open = False

    def log(level, message):
        print("[%s] %s" % (level, message), file=sys.stderr, flush=True)

    def save():
        if latest is not None:
            path = save_image(latest, args.save_dir, count)
            print("saved %s" % path, flush=True)

    def stop():
        nonlocal quit_requested
        if args.duration is not None and time.monotonic() - started >= args.duration:
            return True
        if cv2 is not None and window_open:
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                quit_requested = True
            elif key == ord("s"):
                save()
            elif key == ord("r"):
                transport.request_eeprom()
        return quit_requested

    if args.capture:
        transport = ReplayTransport(args.capture, decoder, replay_rate=args.replay_rate,
                                    loop=args.loop, stop=stop, log=log)
    else:
        transport = SerialTransport(args.port, decoder, baud=args.baud, stop=stop, log=log)
    samples = transport.samples()
    try:
        for sample in samples:
            if stop():
                break
            latest = core.render_image(sample, scale=args.scale,
                                       tmin=args.tmin, tmax=args.tmax,
                                       hotspot=find_hotspot(sample.temps))
            count += 1
            if cv2 is not None:
                cv2.imshow("MLX90640 (q quit, s save, r EEPROM)", latest)
                window_open = True
            if args.save_every and count % args.save_every == 0:
                save()
    finally:
        samples.close()
        transport.close()
        if cv2 is not None and window_open:
            cv2.destroyAllWindows()
    return 0


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        return run(args)
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print("MLX90640 error: %s" % exc, file=sys.stderr)
        return 1
