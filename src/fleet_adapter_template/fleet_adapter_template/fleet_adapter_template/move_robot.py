#!/usr/bin/env python3
import time
import yaml

from RobotClientAPI import RobotAPI
with open("../config.yaml", "r") as f:
        config_yaml = yaml.safe_load(f)

robot_api = RobotAPI(config_yaml["fleet_manager"])

robot_api.navigate2(145)

time.sleep(0.25)
