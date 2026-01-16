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

import logging
import math
import uuid
import networkx as nx

logger = logging.getLogger(__name__)


# Path Planning Functions


def calculate_euclidean_distance(node_u: dict, node_v: dict) -> float:
    """
    Calculate Euclidean distance between two nodes.

    Args:
        node_u: First node data with 'x' and 'y' coordinates
        node_v: Second node data with 'x' and 'y' coordinates

    Returns:
        Euclidean distance between nodes
    """
    dx = node_u["x"] - node_v["x"]
    dy = node_u["y"] - node_v["y"]
    return math.sqrt(dx * dx + dy * dy)


def heuristic(u: str, v: str, graph: nx.DiGraph) -> float:
    return calculate_euclidean_distance(graph.nodes[u], graph.nodes[v])


def calculate_path(
    graph: nx.DiGraph, start_node_name: str, goal_node_name: str
) -> list[str] | None:
    """
    Calculate optimal path between two nodes using A* algorithm.

    Args:
        graph: NetworkX DiGraph with nodes and edges
        start_node_name: Name of the starting node
        goal_node_name: Name of the goal node

    Returns:
        List of node names representing the path, or None if no path exists
    """
    if start_node_name is None or goal_node_name is None:
        return None

    if start_node_name not in graph:
        return None

    if goal_node_name not in graph:
        return None

    try:
        # Use A* algorithm with Euclidean distance heuristic
        path = nx.astar_path(
            graph,
            start_node_name,
            goal_node_name,
            heuristic=lambda u, v: heuristic(u, v, graph),
            weight="weight",
        )
        return path
    except nx.NetworkXNoPath:
        logger.error(f"No path found from {start_node_name} to {goal_node_name}")
        return None
    except Exception as e:
        logger.error(f"Error calculating path: {e}")
        return None


# VDA5050 Mapping Functions


def create_vda5050_edge(
    graph: nx.DiGraph, current_node_name: str, next_node_name: str, sequence_id: int
) -> dict:
    """
    Create a VDA5050 edge from two nodes.

    Args:
        graph: NetworkX DiGraph with edge attributes (speed_limit, etc.)
        current_node_name: Name of the current node
        next_node_name: Name of the next node
        sequence_id: Sequence ID for the edge

    Returns:
        VDA5050 edge dict
    """
    current_node = graph.nodes[current_node_name]
    next_node = graph.nodes[next_node_name]

    is_backward = is_reversed_node(next_node)

    # Determine direction based on coordinate change
    direction = ""
    is_vertical = math.fabs(current_node["x"] - next_node["x"]) < math.fabs(
        current_node["y"] - next_node["y"]
    )
    if is_vertical:
        direction = "Y-" if current_node["y"] > next_node["y"] else "Y+"
    else:
        direction = "X-" if current_node["x"] > next_node["x"] else "X+"

    # Get speed limit from graph edge attributes
    edge_data = graph.get_edge_data(current_node_name, next_node_name)
    speed_limit = edge_data.get("speed_limit", 12) if edge_data else 12

    return {
        "edgeId": f"{next_node_name if is_backward else current_node_name}{current_node_name if is_backward else next_node_name}",
        "sequenceId": sequence_id,
        "released": True,
        "startNodeId": current_node_name,
        "endNodeId": next_node_name,
        "actions": [],
        "maxSpeed": speed_limit * (-1 if is_backward else 1),
        "direction": direction,
    }


def create_vda5050_order(
    graph: nx.DiGraph, robot_name: str, path: list[str]
) -> dict | None:
    """
    Create a VDA5050 order from a pre-calculated path.

    Args:
        graph: NetworkX DiGraph with edge attributes (speed_limit, etc.)
        robot_name: Name of the robot
        path: List of node names representing the path

    Returns:
        VDA5050 order dict, or None if path is invalid
    """
    if path is None or len(path) == 0:
        logger.warning("Path is None or empty")
        return None

    logger.info(f"Creating order for path: {path}")

    order_nodes = []
    order_edges = []

    for i in range(len(path)):
        order_nodes.append(
            {"nodeId": path[i], "sequenceId": i * 2, "released": True, "actions": []}
        )

        if i + 1 < len(path):
            order_edges.append(
                create_vda5050_edge(graph, path[i], path[i + 1], i * 2 + 1)
            )

    order = {
        "version": "2.0.0",
        "manufacturer": "AUBOT",
        "serialNumber": robot_name,
        "orderId": str(uuid.uuid4()),
        "orderUpdateId": 0,
        "nodes": order_nodes,
        "edges": order_edges,
    }
    return order


def is_reversed_node(node: dict) -> bool:
    return node.get("is_parking_spot", False) or node.get("is_charger", False)


def calculate_yaw(x1, y1, x2, y2, reverse=False):
    dx = x2 - x1
    dy = y2 - y1
    yaw = math.atan2(dy, dx)
    if reverse:
        yaw += math.pi
    yaw = (yaw + math.pi) % (2 * math.pi) - math.pi
    return yaw
