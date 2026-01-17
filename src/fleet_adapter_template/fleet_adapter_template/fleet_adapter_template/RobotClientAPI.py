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


"""
The RobotAPI class is a wrapper for API calls to the robot. Here users
are expected to fill up the implementations of functions which will be used
by the RobotCommandHandle. For example, if your robot has a REST API, you
will need to make http request calls to the appropriate endpoints within
these functions.
"""

from datetime import datetime, timezone
import json
import logging
from time import time
import networkx as nx
import paho.mqtt.client as mqtt
from .utils import create_vda5050_order, update_vda5050_order

logger = logging.getLogger(__name__)


class RobotAPI:
    # The constructor below accepts parameters typically required to submit
    # http requests. Users should modify the constructor as per the
    # requirements of their robot's API
    def __init__(self, config_yaml, graph: nx.DiGraph):
        self.mqtt_broker = config_yaml["mqtt_broker"]
        self.mqtt_topic = config_yaml["mqtt_topic"]
        self.mqtt_client_id = config_yaml["mqtt_client_id"]
        self.mqtt_username = config_yaml["mqtt_username"]
        self.mqtt_password = config_yaml["mqtt_password"]
        self.mqtt_qos = config_yaml["mqtt_qos"]
        self.mqtt_retain = config_yaml["mqtt_retain"]
        self.mqtt_keepalive = config_yaml["mqtt_keepalive"]
        self.timeout = 5.0
        self.debug = False
        self.robot_state_data = {}
        self.robot_orders = {}
        self.goal_qr = {}
        self.graph = graph
        self.task_order_data = {}
        self.client = mqtt.Client(self.mqtt_client_id)
        self.client.on_message = self.on_message
        self.client.on_connect = self.on_connect
        self.client.username_pw_set(self.mqtt_username, self.mqtt_password)
        self.client.connect(self.mqtt_broker, keepalive=self.mqtt_keepalive)
        self.client.loop_start()

    def check_connection(self):
        return self.client.is_connected()

    def localize(
        self,
        robot_name: str,
        pose,
        map_name: str,
    ):
        """Request the robot to localize on target map. This
        function should return True if the robot has accepted the
        request, else False"""
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return False

    def navigate(
        self, robot_name: str, path, task_id: str, map_name: str, speed_limit=0.0
    ):
        if path is None or len(path) == 0:
            return False
        if len(path) == 1:
            return True

        goal_qr = path[-1]
        self.goal_qr[robot_name] = goal_qr
        order = None

        if task_id in self.task_order_data:
            order = self.task_order_data[task_id]
            existing_path = [node["nodeId"] for node in order.get("nodes", [])]
            if existing_path == path:
                logger.info(
                    f"Allow navigate for {robot_name}: path is already in order but not send order"
                )
                return True
            order = update_vda5050_order(self.graph, order, path)
        else:
            order = create_vda5050_order(self.graph, robot_name, path)
        self.task_order_data[task_id] = order
        if order is None:
            return False

        topic = f"{self.mqtt_topic}/{robot_name}/order"
        try:
            self.client.publish(topic, json.dumps(order))
            self.robot_orders[robot_name] = order
            return True
        except Exception as e:
            logger.error(f"Failed to publish order: {e}")
            return False

    def start_activity(self, robot_name: str, activity: str, label: str):
        """Request the robot to begin a process. This is specific to the robot
        and the use case. For example, load/unload a cart for Deliverybot
        or begin cleaning a zone for a cleaning robot.
        Return True if process has started/is queued successfully, else
        return False"""
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return False

    def stop(self, robot_name: str):
        stop_action = {
            "actionId": str(mqtt.uuid.uuid4()),
            "actionType": "cancelOrder",
            "blockingType": "HARD",
        }

        instant_action = {
            "headerId": int(time()),
            "timestamp": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            "version": "2.0.0",
            "manufacturer": "OSRF",
            "serialNumber": robot_name,
            "actions": [stop_action],
        }
        self.client.publish(
            f"{self.mqtt_topic}/{robot_name}/instantActions", json.dumps(instant_action)
        )
        return True

    def get_robot_order(self, robot_name: str):
        return self.robot_orders.get(robot_name, None)

    def is_command_completed(self, robot_name: str):
        robot_errors = self.robot_state_data.get(robot_name, {}).get("errors", [])
        if len(robot_errors) > 0:
            for error in robot_errors:
                logger.info(f"Robot {robot_name} has error: {error}")
                error_level = error.get("errorLevel", None)
                if error_level is not None:
                    return False, error_level

        if self.get_last_node_id(robot_name) == self.goal_qr.get(robot_name, None):
            return True, None
        if robot_name in self.robot_orders:
            if self.get_last_node_id(robot_name) is None:
                return False
            cur_order = self.robot_orders[robot_name]
            if cur_order["nodes"][-1]["nodeId"] == self.get_last_node_id(robot_name):
                del self.robot_orders[robot_name]
                return True, None
        return False, None

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.client.subscribe("aubotagv/2.0.0/AUBOT/#")
        else:
            logger.error(f"MQTT connection failed: {rc}")

    def get_last_node_id(self, robot_name: str):
        return self.robot_state_data.get(robot_name, {}).get("lastNodeId", None)

    def get_distance_since_last_node(self, robot_name: str):
        return min(
            self.robot_state_data.get(robot_name, {}).get("distanceSinceLastNode", 0), 1
        )

    def get_node_states(self, robot_name: str):
        return self.robot_state_data.get(robot_name, {}).get("nodeStates", None)

    def get_battery_charge(self, robot_name: str):
        return (
            self.robot_state_data.get(robot_name, {})
            .get("batteryState", {})
            .get("batteryCharge", 0)
        )

    def on_message(self, client, userdata, message):
        payload = json.loads(message.payload.decode())
        robot_name = (
            payload["serialNumber"] if payload and "serialNumber" in payload else None
        )
        if robot_name is None:
            return
        if message.topic.endswith("/state"):
            self.handle_state_message(message.payload.decode())
        elif message.topic.endswith("/order"):
            self.handle_order_message(message.payload.decode())

    def handle_order_message(self, payload):
        try:
            msg = json.loads(payload)
            robot_name = msg.get("serialNumber", None)
            if robot_name is not None:
                self.robot_orders[robot_name] = msg
        except Exception as e:
            logger.error(f"Failed to handle order message: {e}")
            return

    def handle_state_message(self, payload):
        try:
            msg = json.loads(payload)
            robot_name = msg.get("serialNumber", None)
            if robot_name is not None:
                self.robot_state_data[robot_name] = msg
        except Exception as e:
            logger.error(f"Failed to handle state message: {e}")
            return
