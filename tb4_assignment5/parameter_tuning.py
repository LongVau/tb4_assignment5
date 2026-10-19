"""Optional lower-load TurtleBot4 Humble settings for WSL2 simulation.

Copies the installed, version-matched TurtleBot4 YAMLs rather than replacing
Nav2's full parameter files with incomplete overrides.
"""

from copy import deepcopy
from pathlib import Path
import tempfile

import yaml


def tune_wsl_params(localization, navigation):
    """Return tuned YAML dictionaries. Inputs are never mutated."""
    loc = deepcopy(localization)
    nav = deepcopy(navigation)

    amcl = loc['amcl']['ros__parameters']
    amcl['min_particles'] = min(int(amcl.get('min_particles', 500)), 250)
    amcl['max_particles'] = min(int(amcl.get('max_particles', 2000)), 1000)
    amcl['max_beams'] = min(int(amcl.get('max_beams', 60)), 40)
    # Do not enlarge AMCL transform_tolerance to hide multi-second stale scans.

    ctrl = nav['controller_server']['ros__parameters']
    ctrl['controller_frequency'] = 10.0
    if 'FollowPath' in ctrl and 'transform_tolerance' in ctrl['FollowPath']:
        # A little extra margin for minor WSL scheduling jitter, not seconds.
        ctrl['FollowPath']['transform_tolerance'] = 0.35

    local = nav['local_costmap']['local_costmap']['ros__parameters']
    local['update_frequency'] = 3.0
    local['publish_frequency'] = 1.0
    if 'voxel_layer' in local:
        local['voxel_layer']['publish_voxel_map'] = False

    glob = nav['global_costmap']['global_costmap']['ros__parameters']
    glob['publish_frequency'] = 0.5
    return loc, nav


def build_wsl_parameter_files(nav_share):
    """Write two complete tuned parameter YAMLs to a temporary directory."""
    config = Path(nav_share) / 'config'
    with (config / 'localization.yaml').open(encoding='utf-8') as f:
        localization = yaml.safe_load(f)
    with (config / 'nav2.yaml').open(encoding='utf-8') as f:
        navigation = yaml.safe_load(f)
    loc, nav = tune_wsl_params(localization, navigation)

    folder = Path(tempfile.mkdtemp(prefix='tb4_assignment5_wsl_'))
    loc_file = folder / 'localization.yaml'
    nav_file = folder / 'nav2.yaml'
    for filename, data in ((loc_file, loc), (nav_file, nav)):
        with filename.open('w', encoding='utf-8') as out:
            yaml.safe_dump(data, out, sort_keys=False)
    return str(loc_file), str(nav_file)
