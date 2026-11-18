"""Run five small, separated box-to-rack tasks in the existing warehouse."""
import sys
from pathlib import Path
from environment_cdd_project import main

if __name__=='__main__':
    sys.argv.extend(['--small-demo','--fast-cleanup','--perception-backend','off','--tasks-per-robot','0'])
    if '--report' not in sys.argv:
        sys.argv.extend(['--report',str(Path(__file__).with_name('scenes')/'small_cleanup_report.json')])
    raise SystemExit(main())
