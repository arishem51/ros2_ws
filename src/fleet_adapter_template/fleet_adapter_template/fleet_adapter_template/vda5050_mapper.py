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
    def create_order(self, current_node, next_node):
        if current_node is None or next_node is None:
            return None
        next_node_props = next_node["props"]
        current_node_name = current_node["props"]["name"]
        next_node_name = next_node_props["name"]
        order = {
            "version": "2.0.0",
            "manufacturer": "AUBOT",
            "serialNumber": "VAGV1",
            "orderId": str(uuid.uuid4()),
            "orderUpdateId": 0,
            "nodes": [{
                "nodeId": current_node_name,
                "sequenceId": 0,
                "released": True,
                "actions": []
            }, {
                "nodeId": next_node_name,
                "sequenceId": 2,
                "released": True,
                "actions": []
            }],
            "edges": [self._create_edge(current_node, next_node, 1)]
        }
        return order