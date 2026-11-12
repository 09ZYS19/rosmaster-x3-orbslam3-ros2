#!/usr/bin/env python3
"""Inspect saved real RGB-D runs without ROS, NumPy, or reference ground truth."""
import argparse
import csv
from collections import Counter, defaultdict
import math
from pathlib import Path
import statistics


def percentile(values, fraction):
    ordered = sorted(values)
    if not ordered:
        return None
    position = (len(ordered) - 1) * fraction
    low = math.floor(position)
    high = math.ceil(position)
    return ordered[low] + (ordered[high] - ordered[low]) * (position - low)


def read_tum(path):
    rows = []
    if not path.is_file():
        return rows
    for line_number, line in enumerate(path.read_text().splitlines(), 1):
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        values = [float(value) for value in line.split()]
        if len(values) != 8 or not all(math.isfinite(value) for value in values):
            raise ValueError(f'{path.name}:{line_number}: expected 8 finite TUM values')
        if not any(values[4:8]):
            raise ValueError(f'{path.name}:{line_number}: zero quaternion')
        rows.append(values)
    return rows


def geometry(positions):
    return {
        'points': len(positions),
        'span_xyz_m': [max(p[i] for p in positions) - min(p[i] for p in positions)
                       for i in range(3)],
        'endpoint_distance_m': math.dist(positions[0], positions[-1]),
        'estimated_length_m': sum(math.dist(a, b) for a, b in zip(positions, positions[1:])),
    }


def analyze(directory):
    directory = Path(directory)
    summary = {}
    for line in (directory / 'summary.txt').read_text().splitlines():
        key, value = line.split('=', 1)
        summary[key.strip()] = int(value)
    for key in ('processed', 'tracked', 'paired', 'rejected', 'overwritten', 'failed'):
        if key not in summary or summary[key] < 0:
            raise ValueError(f'Missing or invalid summary field: {key}')
    with (directory / 'frames.csv').open(newline='') as stream:
        reader = csv.DictReader(stream)
        required = {'tracking_state', 'valid_depth_fraction', 'timestamp_ns', 'rgb_depth_dt_ms'}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError('frames.csv does not contain the expected real-input columns')
        frames = list(reader)
    states = [int(row['tracking_state']) for row in frames]
    stamps = [int(row['timestamp_ns']) for row in frames]
    offsets = [abs(float(row['rgb_depth_dt_ms'])) for row in frames]
    depths = [float(row['valid_depth_fraction']) for row in frames]
    if not all(math.isfinite(v) for v in offsets + depths):
        raise ValueError('Non-finite frame statistic')
    if not all(0 <= value <= 1 for value in depths):
        raise ValueError('Invalid depth fraction')
    intervals = [(b - a) / 1e9 for a, b in zip(stamps, stamps[1:])]
    issues = []
    if summary['failed']:
        issues.append('SLAM recorded failed != 0')
    if not frames:
        issues.append('No processed frames')
    if len(frames) != summary['processed']:
        issues.append('frames.csv count differs from processed')
    if states.count(2) != summary['tracked']:
        issues.append('state=2 count differs from tracked')
    if not 0 <= summary['tracked'] <= summary['processed'] <= summary['paired']:
        issues.append('Invalid tracked/processed/paired counts')
    if any(value <= 0 for value in intervals):
        issues.append('Processed-frame timestamps are not strictly increasing')
    longest = consecutive = 0
    for state in states:
        consecutive = 0 if state == 2 else consecutive + 1
        longest = max(longest, consecutive)
    trajectory = read_tum(directory / 'CameraTrajectory.txt')
    keyframes = read_tum(directory / 'KeyFrameTrajectory.txt')
    if not trajectory:
        issues.append('Camera trajectory is missing or empty')
    if not keyframes:
        issues.append('Keyframe trajectory is missing or empty')
    span = (stamps[-1] - stamps[0]) / 1e9 if len(stamps) > 1 else 0.0
    groups = defaultdict(list)
    sidecar = directory / 'CameraTrajectory.txt.maps.csv'
    if sidecar.is_file():
        with sidecar.open(newline='') as stream:
            for row in csv.DictReader(stream):
                point = tuple(float(row[key]) for key in ('x', 'y', 'z'))
                if not all(math.isfinite(value) for value in point):
                    raise ValueError('Non-finite map-aware trajectory position')
                groups[str(int(row['map_id']))].append(point)
        if sum(len(points) for points in groups.values()) != len(trajectory):
            issues.append('Map sidecar count differs from TUM trajectory; geometry omitted')
            groups.clear()
    return {
        'directory': str(directory.resolve()),
        'summary': summary,
        'issues': issues,
        'tracked_fraction': summary['tracked'] / summary['processed'] if summary['processed'] else None,
        'state_counts': dict(sorted(Counter(states).items())),
        'longest_non_ok_frames': longest,
        'input_span_s': span,
        'records_per_input_second': (len(frames) - 1) / span if span > 0 else None,
        'max_input_gap_s': max(intervals) if intervals else None,
        'sync_median_ms': statistics.median(offsets) if offsets else None,
        'sync_p95_ms': percentile(offsets, 0.95),
        'sync_max_ms': max(offsets) if offsets else None,
        'mean_valid_depth': statistics.mean(depths) if depths else None,
        'trajectory_rows': len(trajectory),
        'keyframe_rows': len(keyframes),
        'maps': {key: geometry(points) for key, points in sorted(groups.items())},
    }


def format_report(data):
    def number(value):
        return '不适用' if value is None else f'{value:.3f}'

    summary = data['summary']
    ratio = data['tracked_fraction']
    lines = [
        f"结果目录：{data['directory']}",
        '检查范围：运行与轨迹统计；没有参考真值，不计算 ATE/APE、RPE 及其 RMSE。',
        f"文件与计数核对：{'需检查' if data['issues'] else '通过'}（不代表精度通过）",
        f"failed={summary['failed']} processed={summary['processed']} tracked={summary['tracked']}",
        '已处理帧的跟踪比例：' + ('不适用' if ratio is None else f'{ratio:.2%}'),
        f"paired={summary['paired']} rejected={summary['rejected']} overwritten={summary['overwritten']}",
        f"逐帧状态计数：{data['state_counts']}（2=正常，3=暂时丢失，4=丢失）",
        f"最长连续非正常状态：{data['longest_non_ok_frames']} 个已处理帧",
        f"已处理图像的时间跨度：{number(data['input_span_s'])} 秒（不含加载与未记录时段）",
        f"图像时间轴上的平均记录频率：{number(data['records_per_input_second'])} 帧/秒",
        f"相邻处理记录的最大图像时间间隔：{number(data['max_input_gap_s'])} 秒",
        f"RGB-D 时间戳绝对差 median/P95/max：{number(data['sync_median_ms'])} / "
        f"{number(data['sync_p95_ms'])} / {number(data['sync_max_ms'])} 毫秒",
        f"平均有效深度像素比例：{number(data['mean_valid_depth'])}",
        f"相机轨迹 / 关键帧行数：{data['trajectory_rows']} / {data['keyframe_rows']}",
    ]
    for issue in data['issues']:
        lines.append('需检查：' + issue)
    if data['maps']:
        lines.append(f"保存轨迹的最终所属地图数：{len(data['maps'])}；不同地图不拼接计算距离。")
        for map_id, stats in data['maps'].items():
            spans = ', '.join(number(value) for value in stats['span_xyz_m'])
            lines.append(f"地图 {map_id}：{stats['points']} 帧；XYZ 跨度=({spans}) 米；"
                         f"起终点估计距离={number(stats['endpoint_distance_m'])} 米；"
                         f"估计累计长度={number(stats['estimated_length_m'])} 米")
    else:
        lines.append('没有可核对的地图编号记录，未计算可能跨坐标系的距离。')
    lines.extend([
        '频率和时间间隔来自图像时间戳，不是算法计算耗时、CPU 性能或窗口端到端延迟。',
        '跟踪比例只针对已处理帧；rejected/overwritten 不能直接当成跟踪丢失。',
        '时间戳接近不证明物理曝光同步；有效深度比例不证明几何配准准确。',
        '长度/跨度是算法估计，可能受抖动影响；仅真实返回起点的试验才可参考起终点差。',
        '本报告不验证回环成功、地图精度或 Atlas 保存/加载。',
    ])
    return '\n'.join(lines) + '\n'


def save_report(directory, report):
    number = 1
    while True:
        name = 'evaluation.txt' if number == 1 else f'evaluation_{number}.txt'
        path = Path(directory) / name
        try:
            with path.open('x', encoding='utf-8') as stream:
                stream.write(report)
            return path
        except FileExistsError:
            number += 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path, help='Explicit saved run directory; never selects latest')
    parser.add_argument('--save', action='store_true', help='Save evaluation.txt (or evaluation_2.txt, etc.) without overwriting')
    args = parser.parse_args()
    try:
        data = analyze(args.directory)
        report = format_report(data)
        print(report, end='')
        if args.save:
            path = save_report(args.directory, report)
            print(f'报告已保存：{path}')
        return 1 if data['issues'] else 0
    except (OSError, ValueError, KeyError, csv.Error) as error:
        parser.exit(2, f'无法评估：{error}\n请先正常停止 SLAM，并提供正确的完整结果目录。\n')


if __name__ == '__main__':
    raise SystemExit(main())
