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

import json
import logging
import paho.mqtt.client as mqtt

logger = logging.getLogger(__name__)


class RobotAPI:
    # The constructor below accepts parameters typically required to submit
    # http requests. Users should modify the constructor as per the
    # requirements of their robot's API
    def __init__(self, config_yaml):
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

        self.client.on_message = self.on_message
        self.client.on_connect = self.on_connect

        self.client.username_pw_set(self.mqtt_username, self.mqtt_password)
        self.client.connect(self.mqtt_broker, keepalive=self.mqtt_keepalive)
        self.client.loop_start()
        self.node_positions = {
            "0189": [12.321385502772205, -0.41162853141200134, 0],
            "0190": [12.333780701122814, -1.3245925853019644, 0],
            "0191": [12.325517235555742, -2.254064958917136, 0],
            "0467": [13.250876487796312, -2.2499332261336003, 0],
            "0468": [13.250876487796312, -1.3328560508690361, 0],
            "1011": [14.192744059762092, -1.341119516436108, 0],
            "1012": [14.172792629383936, -0.41270799313022244, 0],
            "0099": [13.245237230889053, -0.3999591780098525, 0],
            "1010": [14.200318903198575, -2.264617627783194, 0],
            "0192": [12.316509313631277, -3.1920055235976634, 0],
            "0466": [13.247917273775672, -3.1920055235976634, 0],
            "1104": [14.179306622511133, -3.206261862841756, 0],
            "1103": [14.191925157768958, -4.1255910185874365, 0],
            "0465": [13.249759803260222, -4.117681169790127, 0],
            "0193": [12.315504297548793, -4.133519478793681, 0],
            "0194": [12.315504297548793, -5.059846524298864, 0],
            "0195": [12.325405567102132, -5.984721879907129, 0],
            "0196": [12.308431962153552, -6.907494146305755, 0],
            "0197": [12.308431962153552, -7.869871490917736, 0],
            "0094": [12.319747698785939, -8.781309409275043, 0],
            "0095": [13.231185617143245, -8.781309409275043, 0],
            "0461": [13.259493570133145, -7.858555754285351, 0],
            "0462": [13.24250135377563, -6.9301256195705285, 0],
            "0463": [13.25370542195441, -5.990137799907169, 0],
            "0464": [13.25370542195441, -5.059846524298864, 0],
            "1102": [14.19588938787208, -5.0677749845051085, 0],
            "1101": [14.19983500656627, -5.998066260113413, 0],
            "1100": [14.187905093439031, -6.935802099295657, 0],
        }

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
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
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
        robot_name = payload['serialNumber']
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
