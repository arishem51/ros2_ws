# Requirements
- Docker Desktop up and running
- VSCode
- VSCode extension [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
- XLaunch [source](https://sourceforge.net/projects/vcxsrv/)

# Containerized development
1. Clone base repo
    ```
    git clone https://github.com/hai-dchu/ros2_ws.git
    ```

2. Open in VSCode
	- Run "Dev containers: Reopen in Container" (`Ctrl+Shift+P` to find the command)

3. Setup XLaunch
	- Install and open XLaunch
	- Keep default options

4. In VSCode terminal
	- Run 
        ```
        ros2 launch rmf_demos_gz office.launch.xml
        ```

# Demo
In one terminal, run:
```bash
ros2 launch fleet_adapter_template demo.launch.xml
```

In a new terminal, run:
```bash
python3 -m fleet_adapter_template.fleet_adapter \
    --config_file /home/admin/ros2_ws/build/fleet_adapter_template/config.yaml \
    --nav_graph /home/admin/ros2_ws/build/fleet_adapter_template/maps/0.yaml
```

To send command to the RMF core, run:
```bash
# Go to a predefined point on the map
ros2 run fleet_adapter_template dispatch_go_to_place --fleet AUBOT_AGV --robot VAGV1 --place qr_0465
```