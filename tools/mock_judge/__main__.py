"""Run with python3 -m tools.mock_judge from the workspace root."""
import argparse
import sys
from pathlib import Path

from .app import create_app


def positive(value):
    try:
        parsed = int(value)
    except ValueError:
        raise argparse.ArgumentTypeError("必须为正整数")
    if parsed <= 0:
        raise argparse.ArgumentTypeError("必须为正整数")
    return parsed


def main():
    parser = argparse.ArgumentParser(description="智慧社区模拟裁判台（非官方评分，仅可信局域网联调）")
    parser.add_argument("--host", default="127.0.0.1", help="默认仅本机；小车接入显式设置0.0.0.0")
    parser.add_argument("--port", type=positive, default=8080)
    parser.add_argument("--data-dir", default=str(Path.home() / ".local/share/bit_ws/mock_judge"))
    parser.add_argument("--max-request-mib", type=positive, default=32)
    parser.add_argument("--max-images", type=positive, default=16)
    parser.add_argument("--max-image-mib", type=positive, default=8)
    parser.add_argument("--max-image-pixels", type=positive, default=20_000_000)
    args = parser.parse_args()
    if args.port > 65535:
        parser.error("端口必须在1至65535之间")
    try:
        app = create_app(dict(DATA_DIR=args.data_dir,
                              MAX_CONTENT_LENGTH=args.max_request_mib * 1024 * 1024,
                              MAX_IMAGES=args.max_images,
                              MAX_IMAGE_BYTES=args.max_image_mib * 1024 * 1024,
                              MAX_IMAGE_PIXELS=args.max_image_pixels))
    except Exception as error:
        print("启动失败：%s" % error, file=sys.stderr)
        return 1
    store = app.extensions["judge_store"]
    print("模拟裁判台（非官方评分）。数据目录：%s" % store.root, flush=True)
    print("管理令牌保存在：%s（网页输入令牌后可管理；请勿分享）" % (store.root / "admin_token"), flush=True)
    if args.host not in ("127.0.0.1", "localhost", "::1"):
        print("警告：已启用网络访问。上传和只读接口无认证，仅限可信局域网。", flush=True)
    app.run(host=args.host, port=args.port, debug=False, use_reloader=False, threaded=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
