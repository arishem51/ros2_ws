from collections import defaultdict, deque
import logging
import math
import uuid

logger = logging.getLogger(__name__)

class Vda5050Mapper:
    def __init__(self):
        pass
    def _create_edge(self, current_node, next_node, sequence_id):
        next_node_props = next_node["props"]
        current_node_name = current_node["props"]["name"]
        next_node_name = next_node_props["name"]
        is_backward = 'is_charger' in next_node_props or 'is_parking_spot' in next_node_props
        direction = ""
        is_vertical = math.fabs(current_node["x"] - next_node["x"]) < math.fabs(current_node["y"] - next_node["y"])
        if is_vertical:
            direction = "Y-" if current_node["y"] > next_node["y"] else "Y+"
        else:
            direction = "X-" if current_node["x"] > next_node["x"] else "X+"
        return {
                "edgeId": f"{next_node_name if is_backward else current_node_name}{current_node_name if is_backward else next_node_name}",
                "sequenceId": sequence_id,
                "released": True,
                "startNodeId": current_node_name,
                "endNodeId": next_node_name,
                "actions": [],
                "maxSpeed" : 12 * (-1 if is_backward else 1),
                "direction" : direction
        }
    def _expand_path(self, start, goal, graph: defaultdict[str, list[str]]):
        queue = deque([[start]])
        visited = set([start])
        while queue:
            path = queue.popleft()
            current = path[-1]
            if current == goal:
                return path
            for neighbor in graph[current]:
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(path + [neighbor])
        return None
    
    def create_order(self, robot_name: str, current_node, next_node, graph: defaultdict[str, list[str]], graphNodes: dict[str, dict]):
        if current_node is None or next_node is None:
            return None
        next_node_props = next_node["props"]
        current_node_name = current_node["props"]["name"]
        next_node_name = next_node_props["name"]

        path = self._expand_path(current_node_name, next_node_name, graph)

        logger.info(f"Path: {path}")

        if path is None:
            return None

        nodes = []
        edges = []
        for i in range(len(path)):
            nodes.append({
                "nodeId": path[i],
                "sequenceId": i * 2,
                "released": True,
                "actions": []
            })
            if i + 1 < len(path):
                edges.append(self._create_edge(graphNodes[path[i]], graphNodes[path[i + 1]], i * 2 + 1))
        
        order = {
            "version": "2.0.0",
            "manufacturer": "AUBOT",
            "serialNumber": robot_name,
            "orderId": str(uuid.uuid4()),
            "orderUpdateId": 0,
            "nodes": nodes,
            "edges": edges
        }
        return order