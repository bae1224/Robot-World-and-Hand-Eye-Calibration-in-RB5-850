import argparse
import sys
from .workflow import checked_config, import_images, record_pose


def main():
    parser = argparse.ArgumentParser(description='RB5 + GoPro HERO13 saved-photo RWHE calibration')
    parser.add_argument('--config', default='hardware/rb5.gopro.json')
    commands = parser.add_subparsers(dest='command', required=True)
    record = commands.add_parser('record-pose', help='Read robot state while you manually take a GoPro photo')
    record.add_argument('--sample-id', required=True)
    record.add_argument('--output', required=True)
    record.add_argument('--duration', type=float, default=10, help='Stationary manual-shutter window in seconds')
    capture = commands.add_parser('capture', help='Press Enter for each photo; pose IDs advance automatically')
    capture.add_argument('--output', default='data/rb5/gopro_raw/robot', help='Robot record directory')
    capture.add_argument('--duration', type=float, default=2, help='Manual-shutter window in seconds')
    importer = commands.add_parser('import', help='Process original GoPro photos paired by sample_id')
    importer.add_argument('--manifest', required=True)
    importer.add_argument('--output', required=True)
    args = parser.parse_args()
    try:
        config = checked_config(args.config)
        if args.command == 'record-pose':
            record_pose(config, args.sample_id, args.output, args.duration)
        elif args.command == 'capture':
            from .interactive import capture_poses
            capture_poses(config, args.output, args.duration)
        else:
            import_images(config, args.manifest, args.output)
    except (ValueError, OSError, RuntimeError, ImportError, KeyError) as error:
        print(f'Error: {error}', file=sys.stderr)
        return 1
    except (KeyboardInterrupt, EOFError):
        print('Stopped. No robot motion commands were sent.', file=sys.stderr)
        return 130
    return 0


if __name__ == '__main__':
    sys.exit(main())
