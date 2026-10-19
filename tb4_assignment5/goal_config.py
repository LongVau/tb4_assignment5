"""Load and validate three map-frame navigation goals."""

from dataclasses import dataclass
import math
from pathlib import Path

import yaml


@dataclass(frozen=True)
class Goal:
    name: str
    x: float
    y: float
    yaw: float


def parse_goals(source):
    """Return exactly three unique goals, or raise a descriptive ValueError.

    Yaw values are radians. Coordinates are measured in the map frame.
    """
    if not isinstance(source, dict) or not isinstance(source.get('goals'), list):
        raise ValueError('goals.yaml must contain a top-level goals: list')
    rows = source['goals']
    if len(rows) != 3:
        raise ValueError('The assignment requires exactly three goals (Goal 1 -> 2 -> 3)')

    result = []
    for i, row in enumerate(rows, start=1):
        if not isinstance(row, dict):
            raise ValueError(f'Goal {i} must be a YAML mapping')
        name = row.get('name', f'Goal {i}')
        if not isinstance(name, str) or not name.strip():
            raise ValueError(f'Goal {i} must have a nonempty name')
        coordinates = {}
        for key in ('x', 'y', 'yaw'):
            val = row.get(key)
            if isinstance(val, bool) or not isinstance(val, (int, float)):
                raise ValueError(
                    f'{name}: {key} must be a finite number in map coordinates; '
                    'replace the null placeholders in config/goals.yaml'
                )
            if not math.isfinite(val):
                raise ValueError(f'{name}: {key} must be finite')
            coordinates[key] = float(val)
        result.append(Goal(name=name.strip(), **coordinates))

    names = [goal.name for goal in result]
    if len(names) != len(set(names)):
        raise ValueError('Goal names must be unique')
    if len({(g.x, g.y) for g in result}) != 3:
        raise ValueError('The three goals must be at different map positions')
    return result


def load_goals(path):
    file_path = Path(path).expanduser()
    if not file_path.is_file():
        raise ValueError(f'Goals file is missing: {file_path}')
    try:
        with file_path.open(encoding='utf-8') as stream:
            contents = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        raise ValueError(f'Invalid YAML in {file_path}: {exc}') from exc
    return parse_goals(contents)


def yaw_quaternion(yaw):
    """Return the z and w terms for planar yaw rotation."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)
