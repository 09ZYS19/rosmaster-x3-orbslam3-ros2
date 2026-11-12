#!/usr/bin/env python3
"""Allocate a real-run number, forward shutdown, then evaluate closed files."""
import fcntl
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile


def allocate_output(parent):
    parent = Path(parent)
    parent.mkdir(parents=True, exist_ok=True)
    # A persistent high-water mark also prevents reuse of a deleted run number.
    # The separate lock remains valid while the counter is atomically replaced.
    with (parent / '.run_number.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        counter = parent / '.last_run_number'
        last = int(counter.read_text().strip()) if counter.exists() else 0
        if last < 0:
            raise ValueError('Invalid saved run counter')
        occupied = [int(path.name) for path in parent.iterdir()
                    if path.name.isascii() and path.name.isdecimal()]
        number = max([last, *occupied]) + 1
        while True:
            # Reserve first: interruptions may leave a gap but never overwrite data.
            with tempfile.NamedTemporaryFile(mode='w', dir=parent,
                                             prefix='.run_number_', delete=False) as stream:
                temporary = Path(stream.name)
                stream.write(f'{number}\n')
                stream.flush()
                os.fsync(stream.fileno())
            try:
                os.replace(temporary, counter)
            finally:
                temporary.unlink(missing_ok=True)
            output = parent / str(number)
            try:
                output.mkdir()
                return output
            except FileExistsError:
                number += 1


def command(root, binary, output, extra):
    return [str(binary), '--ros-args',
            '-r', '__ns:=/zys_orbslam3',
            '-r', '/orbslam3/camera_pose:=/zys_orbslam3/camera_pose',
            '-r', '/orbslam3/tracking_state:=/zys_orbslam3/tracking_state',
            '-p', 'use_sim_time:=false', '-p', 'viewer:=false', '-p', 'input_profile:=real',
            '-p', 'sync_mode:=approximate', '-p', 'max_sync_interval_ms:=20.0',
            '-p', 'depth_scale:=0.001', '-p', 'rgb_topic:=/camera/color/image_raw',
            '-p', 'depth_topic:=/camera/depth/image_raw',
            '-p', 'info_topic:=/camera/color/camera_info', '-p', 'camera_fps:=30.0',
            '-p', f'vocabulary:={root}/src/ORB_SLAM3/Vocabulary/ORBvoc.txt',
            *extra, '-p', f'output:={output}']


def run(root, binary, extra):
    root = Path(root).resolve()
    output = allocate_output(root / 'results/real')
    print(f'SLAM output: {output}', flush=True)
    child = None
    requested_signal = None

    def stop(signum, _frame):
        nonlocal requested_signal
        if requested_signal is not None:
            return
        requested_signal = signum
        print('正在等待 SLAM 退出并保存，随后自动评估，请保持终端打开。', flush=True)
        if child is not None and child.poll() is None:
            try:
                os.killpg(child.pid, signum)
            except ProcessLookupError:
                pass

    previous = {sig: signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        # Isolate the child from terminal Ctrl+C; forward it once, so the runner
        # survives to wait for C++ destructors/streams before reading any files.
        child = subprocess.Popen(command(root, binary, output, extra), cwd=output,
                                 start_new_session=True)
        if requested_signal is not None:
            stop_signal = requested_signal
            requested_signal = None
            stop(stop_signal, None)
        status = child.wait()
        if status:
            print(f'SLAM 异常退出（状态 {status}），如有已保存文件，仅作诊断检查。',
                  file=sys.stderr, flush=True)
        if not (output / 'summary.txt').is_file():
            print(f'未找到保存完成的 summary.txt，未执行评估。请保留日志：{output}',
                  file=sys.stderr)
            return (128 - status if status < 0 else status) or 2
        result = subprocess.run([sys.executable, '-B', str(root / 'scripts/evaluate_real_run.py'),
                                 str(output), '--save'], start_new_session=True)
        print(f'X3_RGBD_EVALUATED: status={result.returncode} output={output}', flush=True)
        return (128 - status if status < 0 else status) or result.returncode
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)


if __name__ == '__main__':
    try:
        raise SystemExit(run(sys.argv[1], sys.argv[2], sys.argv[3:]))
    except (OSError, ValueError) as error:
        raise SystemExit(f'无法启动或完成运行：{error}')
