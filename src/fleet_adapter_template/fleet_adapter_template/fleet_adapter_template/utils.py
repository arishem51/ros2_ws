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
import yaml

logger = logging.getLogger(__name__)


def load_navigation_graph(nav_graph_path: str) -> nx.DiGraph:
    graph = nx.DiGraph()

    with open(nav_graph_path, "r") as f:
        graph_data = yaml.safe_load(f)
        vertices = graph_data["levels"]["L1"]["vertices"]
        lanes = graph_data["levels"]["L1"]["lanes"]

        # First pass: create nodes with coordinates
        for idx, vertex in enumerate(vertices):
            x, y, props = vertex
            name = props.get("name", f"qr_{idx}").replace("qr_", "")
            graph.add_node(name, x=x, y=y, **props)

        for idx, lane in enumerate(lanes):
            u_idx, v_idx, lane_props = (
                lane[0],
                lane[1],
                lane[2] if len(lane) > 2 else {},
            )
            u = vertices[u_idx][2].get("name", f"qr_{u_idx}").replace("qr_", "")
            v = vertices[v_idx][2].get("name", f"qr_{v_idx}").replace("qr_", "")

            u_node = graph.nodes[u]
            v_node = graph.nodes[v]
            distance = math.sqrt(
                (u_node["x"] - v_node["x"]) ** 2 + (u_node["y"] - v_node["y"]) ** 2
            )

            speed_limit = lane_props.get("speed_limit", 12)
            graph.add_edge(u, v, weight=distance, speed_limit=speed_limit)

    return graph


def calculate_euclidean_distance(node_u: dict, node_v: dict) -> float:
    dx = node_u["x"] - node_v["x"]
    dy = node_u["y"] - node_v["y"]
    return math.sqrt(dx * dx + dy * dy)


def heuristic(u: str, v: str, graph: nx.DiGraph) -> float:
    return calculate_euclidean_distance(graph.nodes[u], graph.nodes[v])


def calculate_path(
    graph: nx.DiGraph, start_node_name: str, goal_node_name: str
) -> list[str] | None:
    if start_node_name not in graph:
        return None

    if goal_node_name not in graph:
        return None

    try:
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


def create_vda5050_edge(
    graph: nx.DiGraph, current_node_name: str, next_node_name: str, sequence_id: int
) -> dict:
    current_node = graph.nodes[current_node_name]
    next_node = graph.nodes[next_node_name]

    is_backward = is_reversed_node(next_node)
    direction = ""
    is_vertical = math.fabs(current_node["x"] - next_node["x"]) < math.fabs(
        current_node["y"] - next_node["y"]
    )
    if is_vertical:
        direction = "Y-" if current_node["y"] > next_node["y"] else "Y+"
    else:
        direction = "X-" if current_node["x"] > next_node["x"] else "X+"

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


def update_vda5050_order(graph: nx.DiGraph, order: dict, path: list[str]) -> dict:
    order_nodes = order["nodes"]
    node_in_order = next(
        (node for node in order_nodes if node["nodeId"] == path[0]), None
    )
    if node_in_order is None:
        return order

    sequence_id = node_in_order["sequenceId"]
    order_update_id = order["orderUpdateId"]
    order_nodes = []
    order_edges = []
    for i in range(len(path)):
        order_nodes.append(
            {
                "nodeId": path[i],
                "sequenceId": sequence_id + i * 2,
                "released": True,
                "actions": [],
            }
        )
        if i + 1 < len(path):
            order_edges.append(
                create_vda5050_edge(
                    graph, path[i], path[i + 1], sequence_id + i * 2 + 1
                )
            )
    order.update(
        {
            "nodes": order_nodes,
            "edges": order_edges,
            "orderUpdateId": order_update_id + 1,
        }
    )
    return order


def create_vda5050_order(
    graph: nx.DiGraph, robot_name: str, path: list[str]
) -> dict | None:
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


def logger_info():
    counter = 0
    prev_msg = ""
    should_print = True

    def log(message):
        nonlocal counter, prev_msg, should_print
        if should_print:
            print(message)
            prev_msg = message
            should_print = False
        else:
            if prev_msg == message:
                counter += 1
                if counter > 99:
                    should_print = True
            else:
                counter = 0
                prev_msg = message
                should_print = True
        return (message, counter)

    return log
