
"""Validate paths returned by Nav2 before navigation."""

import math


def path_distance(poses):
    samples = list(poses)

    if len(samples) < 2:
        return 0.0, 0.0

    length = sum(
        math.hypot(bx - ax, by - ay)
        for (ax, ay), (bx, by) in zip(samples[:-1], samples[1:])
    )

    start_x, start_y = samples[0]
    displacement = max(
        math.hypot(x - start_x, y - start_y)
        for x, y in samples[1:]
    )

    return length, displacement


def assess_nav2_path(path, min_travel=0.10):
    samples = []

    for pose in path.poses:
        x = float(pose.pose.position.x)
        y = float(pose.pose.position.y)

        if not math.isfinite(x) or not math.isfinite(y):
            return False, 'planner returned a non-finite path coordinate', 0.0

        samples.append((x, y))

    if not samples:
        return False, 'planner returned 0 poses (empty path)', 0.0

    if len(samples) == 1:
        return False, 'planner returned a single pose (no translational route)', 0.0

    length, displacement = path_distance(samples)

    if length < min_travel or displacement < min_travel:
        return False, (
            f'path has effectively no movement '
            f'(length={length:.3f} m, displacement={displacement:.3f} m)'
        ), length

    return True, f'{len(samples)} poses, {length:.2f} m', length


def close_goal_pairs(goals, warning_distance=0.50):
    pairs = []

    for a, b in zip(goals[:-1], goals[1:]):
        distance = math.hypot(b.x - a.x, b.y - a.y)

        if distance < warning_distance:
            pairs.append((a.name, b.name, distance))

    return pairs
