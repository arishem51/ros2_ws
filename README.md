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
