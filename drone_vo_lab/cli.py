import argparse
from datetime import datetime
from pathlib import Path

def main():
    parser = argparse.ArgumentParser(description="RGB-D visual odometry stress tests in MuJoCo")
    sub = parser.add_subparsers(dest="command", required=True)
    bench = sub.add_parser("benchmark", help="Capture, evaluate, and write a self-contained report")
    bench.add_argument("--output", type=Path, default=Path("runs") / datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))
    #custom scenes can be added and configured here. the default for now are warehouse and low texture.
    bench.add_argument(
        "--scenes",
        nargs="+",
        choices=["warehouse", "low_texture"],
        default=["warehouse", "low_texture"],
    )
    bench.add_argument("--frame-interval", type=int, default=4)
    bench.add_argument("--seeds", nargs="+", type=int, default=[0, 1, 2])
    bench.add_argument("--frames", type=int, default=180)
    bench.add_argument("--fps", type=float, default=15)
    bench.add_argument("--width", type=int, default=320)
    bench.add_argument("--height", type=int, default=240)
    bench.add_argument("--no-video", action="store_true")
    bench.add_argument("--quick", action="store_true", help="One scene, one seed, 90 frames")
    args = parser.parse_args()
    
    effective_frames = min(args.frames, 90) if args.quick else args.frames

    if args.frame_interval <= 0:
        parser.error(" frame interval must be a positive integer")
    if args.frame_interval >= effective_frames:
        parser.error("frame interval must leave atleast 2 frames")
    if args.frames < 8 or args.fps <= 0 or min(args.width, args.height) < 64:
        parser.error("Need at least 8 frames, positive FPS, and image dimensions >= 64")
    if args.width > 1280 or args.height > 720:
        parser.error("Maximum render size is 1280 x 720")
    from .benchmark import run_benchmark

    run_benchmark(
        args.output,
        ["warehouse"] if args.quick else list(dict.fromkeys(args.scenes)),
        [0] if args.quick else list(dict.fromkeys(args.seeds)),
        effective_frames,
        args.fps,
        args.width,
        args.height,
        not args.no_video,
        args.frame_interval,
    )
