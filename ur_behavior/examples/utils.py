"""
Utilities for the demo.
"""

import os
from ament_index_python.packages import get_package_share_directory


def create_world_from_yaml(world_file):
    """ Helper function to get a world file from the data folder. """
    data_folder = os.path.join(
        get_package_share_directory("ur_behavior"), "data")

    from pyrobosim.core import WorldYamlLoader
    loader = WorldYamlLoader()
    return loader.from_yaml(os.path.join(data_folder, world_file + ".yaml"))


def get_domains_folder():
    """ Helper function to get the planning domains folder. """
    return os.path.join(
        get_package_share_directory("ur_behavior"),
        "data", "planning_domains"
    )


def get_pddlstream_mapping_functions():
    """
    Get the PDDLStream mapping functions for this domain.
    
    Instead of creating them from scratch, we can use the default mapping dictionaries
    from pyrobosim and only add/replace the ones we need.
    """
    from pyrobosim.planning.pddlstream.default_mappings import (
        get_stream_map, get_stream_info
    )

    def stream_map_fn(world, robot):
        stream_map = get_stream_map(world, robot)

        # Add new stream map entries
        stream_map["PoseDist"] = lambda p1, p2: p1.get_linear_distance(p2)
        
        return stream_map

    def stream_info_fn():
        return get_stream_info()

    return (stream_info_fn, stream_map_fn)
