"""One status_light per machine: the newest one takes the blink(1) over.

There is one physical light, and a blink(1) opened through libusb can't be opened again until
its holder lets go. A status_light left running (a dashboard restart that lost track of its
idle light, a launch killed without cleanup) kept the light forever, and every later one
logged "no blink(1): open failed". So a starting node stops the others first: SIGINT (they
turn the light off and close the device), then SIGTERM for any that hang.

Other instances are found by their command line (the installed status_light entry point), so
this also catches nodes started by older code. No ROS here, so it is testable on its own.
"""

import os
import signal
import time

ENTRY_POINT = '/lib/status_light/status_light'


def _cmdline(pid, proc='/proc'):
    try:
        with open(f'{proc}/{pid}/cmdline', 'rb') as f:
            return [a.decode(errors='replace') for a in f.read().split(b'\0') if a]
    except OSError:
        return []


def find_others(me=None, proc='/proc'):
    """PIDs of other status_light node processes (not their `ros2 run` / shell wrappers)."""
    me = os.getpid() if me is None else me
    found = []
    for name in os.listdir(proc):
        if not name.isdigit() or int(name) == me:
            continue
        argv = _cmdline(name, proc)
        # The node itself: `python .../lib/status_light/status_light ...` (or exec'd directly).
        if any(a.endswith(ENTRY_POINT) for a in argv[:2]):
            found.append(int(name))
    return sorted(found)


def _alive(pid):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def take_over(me=None, grace_s=3.0, log=print, proc='/proc', kill=os.kill):
    """Stop every other status_light node. -> the pids that were asked to stop."""
    others = find_others(me, proc)
    asked = []
    for pid in others:
        log(f'another status_light (pid {pid}) holds the light; asking it to stop')
        try:
            kill(pid, signal.SIGINT)
            asked.append(pid)
        except ProcessLookupError:
            pass
        except PermissionError:
            log(f'pid {pid} belongs to another user; not stopping it')
    deadline = time.monotonic() + grace_s
    while any(_alive(p) for p in asked) and time.monotonic() < deadline:
        time.sleep(0.1)
    for pid in asked:
        if _alive(pid):
            log(f'pid {pid} ignored SIGINT; sending SIGTERM')
            try:
                kill(pid, signal.SIGTERM)
            except (ProcessLookupError, PermissionError):
                pass
    if asked:
        time.sleep(0.5)  # let the kernel give the USB interface back
    return asked
