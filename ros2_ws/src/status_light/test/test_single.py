"""single.py: finding (and stopping) other status_light nodes, against a fake /proc."""
import signal

from status_light import single


def fake_proc(tmp_path, procs):
    for pid, argv in procs.items():
        d = tmp_path / str(pid)
        d.mkdir()
        (d / 'cmdline').write_bytes(b'\0'.join(a.encode() for a in argv) + b'\0')
    (tmp_path / 'self').mkdir()  # non-numeric entries are skipped
    return str(tmp_path)


NODE = ['/ws/.pixi/envs/l4t/bin/python3.12', '/ws/install/status_light/lib/status_light/status_light',
        '--ros-args', '-p', 'pattern:=idle']
WRAPPER = ['/ws/.pixi/envs/l4t/bin/python', '/ws/.pixi/envs/l4t/bin/ros2', 'run', 'status_light',
           'status_light']
SHELL = ['bash', '-c', 'exec pixi run -e l4t ros2 run status_light status_light']


def test_finds_only_other_node_processes(tmp_path):
    proc = fake_proc(tmp_path, {10: NODE, 11: WRAPPER, 12: SHELL, 13: ['sleep', '5'], 20: NODE})
    assert single.find_others(me=20, proc=proc) == [10]


def test_take_over_signals_each_other_node(tmp_path, monkeypatch):
    proc = fake_proc(tmp_path, {10: NODE, 30: NODE, 11: WRAPPER})
    sent = []
    monkeypatch.setattr(single, '_alive', lambda pid: False)  # they exit at once
    asked = single.take_over(me=99, proc=proc, kill=lambda pid, sig: sent.append((pid, sig)),
                             log=lambda m: None, grace_s=0.1)
    assert asked == [10, 30]
    assert sent == [(10, signal.SIGINT), (30, signal.SIGINT)]


def test_take_over_escalates_to_sigterm(tmp_path, monkeypatch):
    proc = fake_proc(tmp_path, {10: NODE})
    sent = []
    monkeypatch.setattr(single, '_alive', lambda pid: True)  # ignores SIGINT
    single.take_over(me=99, proc=proc, kill=lambda pid, sig: sent.append((pid, sig)),
                     log=lambda m: None, grace_s=0.1)
    assert sent == [(10, signal.SIGINT), (10, signal.SIGTERM)]


def test_nothing_to_do(tmp_path):
    proc = fake_proc(tmp_path, {11: WRAPPER, 20: NODE})
    assert single.take_over(me=20, proc=proc, kill=lambda *a: None, log=lambda m: None) == []
