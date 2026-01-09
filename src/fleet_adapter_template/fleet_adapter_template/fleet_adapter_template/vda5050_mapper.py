import logging
import math
import uuid

logger = logging.getLogger(__name__)

class Vda5050Mapper:
    def __init__(self, lanes: dict[str, list[int]]):
        self.lanes = lanes
    
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
        lane = self.lanes.get(f"{current_node_name}{next_node_name}")
        speed_limit = lane[2].get("speed_limit", 12) if lane is not None else 12 
        return {
                "edgeId": f"{next_node_name if is_backward else current_node_name}{current_node_name if is_backward else next_node_name}",
                "sequenceId": sequence_id,
                "released": True,
                "startNodeId": current_node_name,
                "endNodeId": next_node_name,
                "actions": [],
                "maxSpeed" : speed_limit * (-1 if is_backward else 1),
                "direction" : direction
        }
    
    def create_order(self, robot_name: str, path: list[str], nodes: dict[str, dict]):
        """
        Create a VDA5050 order from a pre-calculated path.
        
        Args:
            robot_name: Name of the robot
            path: List of node names representing the path
            nodes: Dictionary of node data indexed by node name
            
        Returns:
            VDA5050 order dict, or None if path is invalid
        """
        if path is None or len(path) == 0:
            return None

        logger.info(f"Creating order for path: {path}")

        order_nodes = []
        order_edges = []
        for i in range(len(path)):
            order_nodes.append({
                "nodeId": path[i],
                "sequenceId": i * 2,
                "released": True,
                "actions": []
            })
            if i + 1 < len(path):
                order_edges.append(self._create_edge(nodes[path[i]], nodes[path[i + 1]], i * 2 + 1))
        
        order = {
            "version": "2.0.0",
            "manufacturer": "AUBOT",
            "serialNumber": robot_name,
            "orderId": str(uuid.uuid4()),
            "orderUpdateId": 0,
            "nodes": order_nodes,
            "edges": order_edges
        }
        return order