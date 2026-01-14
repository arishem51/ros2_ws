# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import argparse
from rclpy.qos import QoSProfile, qos_profile_system_default
import yaml
import time
import threading
import asyncio
import math

import rclpy
import rclpy.node
from rmf_fleet_msgs.msg import ClosedLanes, LaneRequest

import rmf_adapter
from rmf_adapter import Adapter
import rmf_adapter.easy_full_control as rmf_easy
import logging
import networkx as nx
from .RobotClientAPI import RobotAPI

from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy as Reliability

logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(asctime)s %(message)s')

logger = logging.getLogger(__name__)


def load_navigation_graph(nav_graph_path: str) -> tuple[nx.DiGraph, dict[str, dict]]:
    """
    Load navigation graph from YAML file and create NetworkX graph.
    
    Args:
        nav_graph_path: Path to the navigation graph YAML file
        
    Returns:
        Tuple of (NetworkX DiGraph, nodes dictionary)
    """
    graph = nx.DiGraph()
    nodes = {}
    
    with open(nav_graph_path, 'r') as f:
        graph_data = yaml.safe_load(f)
        vertices = graph_data['levels']['L1']['vertices']
        lanes = graph_data['levels']['L1']['lanes']
        
        # First pass: create nodes with coordinates
        for idx, vertex in enumerate(vertices):
            x, y, props = vertex
            name = props.get('name', f'qr_{idx}').replace('qr_', '')
            nodes[name] = {"x": x, "y": y, "props": {**props, "name": name}}
            graph.add_node(name, x=x, y=y, **props)
        
        # Second pass: create edges with attributes
        for idx, lane in enumerate(lanes):
            u_idx, v_idx, lane_props = lane[0], lane[1], lane[2] if len(lane) > 2 else {}
            u = vertices[u_idx][2].get("name", f'qr_{u_idx}').replace('qr_', '')
            v = vertices[v_idx][2].get("name", f'qr_{v_idx}').replace('qr_', '')
            
            u_node = nodes[u]
            v_node = nodes[v]
            distance = math.sqrt((u_node["x"] - v_node["x"])**2 + (u_node["y"] - v_node["y"])**2)
            
            speed_limit = lane_props.get("speed_limit", 12)
            
            graph.add_edge(u, v, weight=distance, speed_limit=speed_limit)
    
    logger.info(f"Loaded navigation graph with {graph.number_of_nodes()} nodes and {graph.number_of_edges()} edges")
    return graph, nodes


# ------------------------------------------------------------------------------
# Main
# ------------------------------------------------------------------------------
def main(argv=sys.argv):
    # Init rclpy and adapter
    rclpy.init(args=argv)
    rmf_adapter.init_rclcpp()
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog="fleet_adapter",
        description="Configure and spin up the fleet adapter")
    parser.add_argument("-c", "--config_file", type=str, required=True,
                        help="Path to the config.yaml file")
    parser.add_argument("-n", "--nav_graph", type=str, required=True,
                        help="Path to the nav_graph for this fleet adapter")
    parser.add_argument("-s", "--server_uri", type=str, required=False, default="",
                        help="URI of the api server to transmit state and task information.")
    parser.add_argument("-sim", "--use_sim_time", action="store_true",
                        help='Use sim time, default: false')
    args = parser.parse_args(args_without_ros[1:])
    print(f"Starting fleet adapter...")

    config_path = args.config_file
    nav_graph_path = args.nav_graph

    fleet_config = rmf_easy.FleetConfiguration.from_config_files(
        config_path, nav_graph_path
    )
    assert fleet_config, f'Failed to parse config file [{config_path}]'

    # Parse the yaml in Python to get the fleet_manager info
    with open(config_path, "r") as f:
        config_yaml = yaml.safe_load(f)

    # ROS 2 node for the command handle
    fleet_name = fleet_config.fleet_name
    node = rclpy.node.Node(f'{fleet_name}_command_handle')
    adapter = Adapter.make(f'{fleet_name}_fleet_adapter')
    assert adapter, (
        'Unable to initialize fleet adapter. '
        'Please ensure RMF Schedule Node is running'
    )

    adapter.start()
    time.sleep(1.0)

    if args.server_uri == '':
        server_uri = None
    else:
        server_uri = args.server_uri

    fleet_config.server_uri = server_uri

    fleet_handle = adapter.add_easy_fleet(fleet_config)

    # Load navigation graph
    graph, nodes = load_navigation_graph(nav_graph_path)

    # Initialize robot API for this fleet
    fleet_mgr_yaml = config_yaml['fleet_manager']
    api = RobotAPI(fleet_mgr_yaml, graph, nodes)

    robots = {}
    for robot_name in fleet_config.known_robots:
        robot_config = fleet_config.get_known_robot_configuration(robot_name)
        robots[robot_name] = RobotAdapter(
            robot_name, robot_config, node, api, fleet_handle
        )

    update_period = 1.0/config_yaml['rmf_fleet'].get(
        'robot_state_update_frequency', 10.0
    )

    def update_loop():
        asyncio.set_event_loop(asyncio.new_event_loop())
        next_wakeup = time.monotonic()
        loop = asyncio.get_event_loop()
        while rclpy.ok():
            
            jobs = [update_robot(r) for r in robots.values()]
            loop.run_until_complete(asyncio.gather(*jobs))
            next_wakeup += update_period
            sleep_time = next_wakeup - time.monotonic()
                        
            if sleep_time > 0:
                time.sleep(sleep_time)

    update_thread = threading.Thread(target=update_loop, args=())
    update_thread.start()

    connections = ros_connections(node, robots, fleet_handle)
    connections

    # Create executor for the command handle node
    rclpy_executor = rclpy.executors.SingleThreadedExecutor()
    rclpy_executor.add_node(node)

    # Start the fleet adapter
    rclpy_executor.spin()

    # Shutdown
    node.destroy_node()
    rclpy_executor.shutdown()
    rclpy.shutdown()


class RobotAdapter:
    def __init__(
        self,
        name: str,
        configuration,
        node,
        api: RobotAPI,
        fleet_handle
    ):
        self.name = name
        self.execution = None
        self.update_handle = None
        self.configuration = configuration
        self.node = node
        self.api = api
        self.fleet_handle = fleet_handle
        self.task_list = []
        self.task_id = None
        self.log_counter = 0
        self.prev_state = None

    def update(self, state):
        activity_identifier = None
        execution = self.execution
        if execution:
            if self.api.is_command_completed(self.name):
                execution.finished()
                self.execution = None
            else:
                activity_identifier = execution.identifier

        self.update_handle.update(state, activity_identifier)

    def make_callbacks(self):
        callbacks = rmf_easy.RobotCallbacks(
            lambda destination, execution: self.navigate(
                destination, execution
            ),
            lambda activity: self.stop(activity),
            lambda category, description, execution: self.execute_action(
                category, description, execution
            )
        )

        callbacks.localize = lambda estimate, execution: self.localize(
            estimate, execution
        )

        return callbacks

    def localize(self, estimate, execution):
        self.node.get_logger().info(
            f'Commanding [{self.name}] to change map to'
            f' [{estimate.map}]'
        )
        if self.api.localize(self.name, estimate.position, estimate.map):
            self.node.get_logger().info(
                f'Localized [{self.name}] on {estimate.map} '
                f'at position [{estimate.position}]'
            )
            execution.finished()
        else:
            self.node.get_logger().warn(
                f'Failed to localize [{self.name}] on {estimate.map} '
                f'at position [{estimate.position}]. Requesting replanning...'
            )
            if self.update_handle is not None and self.update_handle.more() is not None:
                self.update_handle.more().replan()

    def navigate(self, destination, execution):
        self.execution = execution
        self.node.get_logger().info(
            f'Commanding [{self.name}] to navigate to {destination.position} '
            f'on map [{destination.map}]'
        )

        self.api.navigate(
            self.name,
            destination.position,
            destination.map,
            destination.speed_limit
        )

    def stop(self, activity):
        execution = self.execution
        if execution is not None:
            if execution.identifier.is_same(activity):
                self.execution = None
                self.api.stop(self.name)

    def execute_action(self, category: str, description: dict, execution):
        ''' Trigger a custom action you would like your robot to perform.
        You may wish to use RobotAPI.start_activity to trigger different
        types of actions to your robot.'''
        self.execution = execution
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return


def ros_connections(node, robots, fleet_handle):
    fleet_name = fleet_handle.more().fleet_name
    transeint_qos = QoSProfile(
        history=History.KEEP_LAST,
        depth=1,
        reliability=Reliability.RELIABLE,
        durability=Durability.TRANSIENT_LOCAL,
    )

    close_lanes_pub = node.create_publisher(
        ClosedLanes,
        'closed_lanes',
        qos_profile=transeint_qos
    )

    closed_lanes = set()

    def lane_request_cb(msg):
        if msg.fleet_name and msg.fleet_name != fleet_name:
            print(f'Ignoring lane request for fleet [{msg.fleet_name}]')
            return

        if msg.open_lanes:
            print(f'Opening lanes: {msg.open_lanes}')

        if msg.close_lanes:
            print(f'Closing lanes: {msg.close_lanes}')

        fleet_handle.more().open_lanes(msg.open_lanes)
        fleet_handle.more().close_lanes(msg.close_lanes)

        for lane_idx in msg.close_lanes:
            closed_lanes.add(lane_idx)

        for lane_idx in msg.open_lanes:
            closed_lanes.remove(lane_idx)

        state_msg = ClosedLanes()
        state_msg.fleet_name = fleet_name
        state_msg.closed_lanes = list(closed_lanes)
        close_lanes_pub.publish(state_msg)

    lane_request_sub = node.create_subscription(
        LaneRequest,
        'lane_closure_requests',
        lane_request_cb,
        qos_profile=qos_profile_system_default,
    )
    
    return [
        lane_request_sub,
    ]

    ## todo: task management with updateOrderId instead of creating new order for each navigate calling


# Parallel processing solution derived from
# https://stackoverflow.com/a/59385935
def parallel(f):
    def run_in_parallel(*args, **kwargs):
        return asyncio.get_event_loop().run_in_executor(
            None, f, *args, **kwargs
        )

    return run_in_parallel


@parallel
def update_robot(robot: RobotAdapter):
    data = robot.api.get_data(robot.name)
    if data is None:
        return
    state = rmf_easy.RobotState(
        data.map,
        data.position,
        data.battery_soc
    )

    if robot.update_handle is None:
        robot.update_handle = robot.fleet_handle.add_robot(
            robot.name,
            state,
            robot.configuration,
            robot.make_callbacks()
        )
        return

    robot.update(state)


if __name__ == '__main__':
    main(sys.argv)
