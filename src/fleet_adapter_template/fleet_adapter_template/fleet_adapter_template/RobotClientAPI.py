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


'''
    The RobotAPI class is a wrapper for API calls to the robot. Here users
    are expected to fill up the implementations of functions which will be used
    by the RobotCommandHandle. For example, if your robot has a REST API, you
    will need to make http request calls to the appropriate endpoints within
    these functions.
'''

import datetime
import json
import logging
import math
from time import timezone
import paho.mqtt.client as mqtt
import yaml
from pathlib import Path
from .vda5050_mapper import Vda5050Mapper

logger = logging.getLogger(__name__)


class RobotAPI:
    # The constructor below accepts parameters typically required to submit
    # http requests. Users should modify the constructor as per the
    # requirements of their robot's API
    def __init__(self, config_yaml, nav_graph_path):
        self.mqtt_broker = config_yaml['mqtt_broker']
        self.mqtt_topic = config_yaml['mqtt_topic']
        self.mqtt_client_id = config_yaml['mqtt_client_id']
        self.mqtt_username = config_yaml['mqtt_username']
        self.mqtt_password = config_yaml['mqtt_password']
        self.mqtt_qos = config_yaml['mqtt_qos']
        self.mqtt_retain = config_yaml['mqtt_retain']
        self.mqtt_keepalive = config_yaml['mqtt_keepalive']
        self.timeout = 5.0
        self.debug = False
        self.robot_states = {}
        self.client = mqtt.Client(self.mqtt_client_id)
        self.nav_graph_path = Path(nav_graph_path)
        self.client.on_message = self.on_message
        self.client.on_connect = self.on_connect
        self.client.username_pw_set(self.mqtt_username, self.mqtt_password)
        self.client.connect(self.mqtt_broker, keepalive=self.mqtt_keepalive)
        self.client.loop_start()
        self.node_positions = {}
        with open(nav_graph_path, 'r') as f:
            graph_data = yaml.safe_load(f)
            vertices = graph_data['levels']['L1']['vertices']
            for idx, vertex in enumerate(vertices):
                x,y, props = vertex
                name = props.get('name', f'qr_{idx}').replace('qr_', '')
                self.node_positions[name] = [x, y, 0]
        logger.info(f"node_positions: {self.node_positions}")
        
        # Initialize VDA5050 mapper
        self.vda5050_mapper = Vda5050Mapper()
        logger.info("VDA5050 mapper initialized successfully")

    def check_connection(self):
        ''' Return True if connection to the robot API server is successful '''
        return self.client.is_connected()

    def localize(
        self,
        robot_name: str,
        pose,
        map_name: str,
    ):
        ''' Request the robot to localize on target map. This 
            function should return True if the robot has accepted the 
            request, else False '''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return False
    def position_to_node_id(self, position):
        next_node = None
        min_dist = float('inf')
        for node_id, (x, y, _) in self.node_positions.items():
            dist = math.hypot(x - position[0], y - position[1])
            if next_node is None or dist < min_dist:
                next_node = node_id
                min_dist = dist
        return next_node
    
    def navigate(
        self,
        robot_name: str,
        pose,
        map_name: str,
        speed_limit=0.0
    ):
        ''' Request the robot to navigate to pose:[x,y,theta] where x, y and
            and theta are in the robot's coordinate convention. This function
            should return True if the robot has accepted the request,
            else False '''
        logger.info(f"Navigating to pose: {pose}")
        current_node_id = self.position(robot_name)
        next_node_id = self.position_to_node_id(pose)
        order = self.vda5050_mapper.map_order(current_node_id, next_node_id)
        if order is None:
            logger.error(f"No order found for current node: {current_node_id} and next node: {next_node_id}")
            return False
        logger.info(f"Navigating to node: {next_node_id}")
        topic = f"{self.mqtt_topic}/{robot_name}/order"
        try:
            self.client.publish(topic, json.dumps(order))
            logger.info(f"Published order: {order}")
            return True
        except Exception as e:
            logger.error(f"Failed to publish order: {e}")
            return False
    
    def start_activity(
        self,
        robot_name: str,
        activity: str,
        label: str
    ):
        ''' Request the robot to begin a process. This is specific to the robot
        and the use case. For example, load/unload a cart for Deliverybot
        or begin cleaning a zone for a cleaning robot.
        Return True if process has started/is queued successfully, else
        return False '''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return False

    def stop(self, robot_name: str):
        ''' Command the robot to stop.
            Return True if robot has successfully stopped. Else False. '''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return False

    def position(self, robot_name: str):
        if robot_name not in self.robot_states or "last_node_id" not in self.robot_states[robot_name]:
            return None
        last_node_id = self.robot_states[robot_name]["last_node_id"]
        if(last_node_id in self.node_positions):
            return self.node_positions[last_node_id]
        return None

    def battery_soc(self, robot_name: str):
        if robot_name not in self.robot_states or "battery_soc" not in self.robot_states[robot_name]:
            return None
        return self.robot_states[robot_name]["battery_soc"] / 100.0

    def map(self, robot_name: str):
        return "L1"

    def is_command_completed(self):
        ''' Return True if the robot has completed its last command, else
        return False. '''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return False

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            logger.info("MQTT connected successfully")
            self.client.subscribe("aubotagv/2.0.0/AUBOT/#")
        else:
            logger.error(f"MQTT connection failed: {rc}")
    
    def on_message(self, client, userdata, message):
        payload = json.loads(message.payload.decode())
        robot_name = payload["serialNumber"] if payload and "serialNumber" in payload else None
        if robot_name is None:
            return
        if "batteryState" in payload:
            battery_soc = payload["batteryState"]["batteryCharge"]
            self.robot_states[robot_name] = {
                **self.robot_states.get(robot_name, {}),
                "battery_soc": battery_soc
            }
        if "lastNodeId" in payload:
            last_node_id = payload["lastNodeId"]
            self.robot_states[robot_name] = {
                **self.robot_states.get(robot_name, {}),
                "last_node_id": last_node_id
            }
        

    def get_data(self, robot_name: str):
        map = self.map(robot_name)
        position = self.position(robot_name)
        battery_soc = self.battery_soc(robot_name)
        if not (map is None or position is None or battery_soc is None):
            return RobotUpdateData(robot_name, map, position, battery_soc)
        return None

class RobotUpdateData:
    ''' Update data for a single robot. '''
    def __init__(self,
                 robot_name: str,
                 map: str,
                 position: list[float],
                 battery_soc: float,
                 requires_replan: bool | None = None):
        self.robot_name = robot_name
        self.position = position
        self.map = map
        self.battery_soc = battery_soc
        self.requires_replan = requires_replan
