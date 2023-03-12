#
#  Created by ICraveSleep on 23.02.23.
#

import logging
import os
import os.path
from launch import LaunchDescription
from launch_ros.actions import Node

logger = logging.getLogger('launch')
logger.setLevel(logging.INFO)


def generate_launch_description():
    nodes = []
    log_level = "INFO"  # [DEBUG, INFO, WARN, ERROR]
    package_name = "ivp_control"
    namespace = "ivp"
    executable = "ivp_control"
    node_name = "ivp_control_test"
    node = Node(
        package=package_name,
        namespace=namespace,
        executable=executable,
        parameters=[],
        name=node_name,
        arguments=["--ros-args", "--log-level", f"{get_logger_name(namespace, node_name)}:={log_level}"],
        output="screen",
        emulate_tty=True
    )

    nodes.append(node)

    return LaunchDescription(nodes)


def get_logger_name(namespace: str = None, node_name: str = None):
    if node_name is None:
        logger.warning("No node_name set. setting default name: no_name")
        return "no_name"

    if namespace is None:
        return node_name

    return f"{namespace}.{node_name}"
