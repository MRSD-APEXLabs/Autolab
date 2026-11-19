"""Explicit OT-2 initialization/reset: home and discard any tracked tips."""
import argparse
import json
from pathlib import Path
import time

from gcs_monitoring.ot2_part1 import request


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--state', type=Path, required=True)
    args = parser.parse_args()
    wrapper = 'http://127.0.0.1:8001'
    previous = args.state.read_text() if args.state.exists() else None
    if previous is not None:
        args.state.with_name(f'{args.state.stem}.before-cleanup-{time.time_ns()}.json').write_text(previous)
    state = {'stage': 'cleanup_started', 'wrapper_url': wrapper, 'ssh_host': 'p5090'}

    def save():
        args.state.write_text(json.dumps(state, indent=2) + '\n')

    save()
    try:
        status = request('', wrapper, '/robot/status')
        if not status['connected']:
            # Connecting initializes and homes once; do not home twice.
            print(request('', wrapper, '/robot/connect', {}), flush=True)
        else:
            print(request('', wrapper, '/robot/home', {}), flush=True)
        print(request('', wrapper, '/commands/drop-tips', {}), flush=True)
        state['stage'] = 'prepared'
        save()
        print('Homed, discarded mounted tips, and connected. Ready for Part 1.', flush=True)
    except Exception as exc:
        state.update(stage='needs_inspection', error=str(exc))
        save()
        raise SystemExit(f'Cleanup stopped: {exc}') from exc


if __name__ == '__main__':
    main()
