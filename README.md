# tsu200 - example custom service

This package demonstrates a simple custom ROS1 service (Python).

Service: `Compute` (defined in `srv/Compute.srv`)
- Request: int32 a, int32 b
- Response: int32 sum

Files added:
- `srv/Compute.srv` - service definition
- `scripts/compute_server.py` - Python service server
- `scripts/compute_client.py` - Python client
 - `srv/TaskOpt.srv` - TaskOpt service definition
 - `scripts/taskopt_server.py` - TaskOpt Python service server
 - `scripts/taskopt_client.py` - TaskOpt Python client

Build and run (in WSL Ubuntu 20.04):

1. Make sure the package lists `message_generation` and `message_runtime` deps in `package.xml`, and `message_generation` in `CMakeLists.txt` (see suggested patches in this README below).
2. From your catkin workspace root (`uav_ws`):

```bash
# source your ROS setup and build
source /opt/ros/<distro>/setup.bash; catkin_make; source devel/setup.bash

# run the service server in one terminal
rosrun tsu200 compute_server.py

# in another terminal, call the client
rosrun tsu200 compute_client.py 5 7
# should print: Result: 12
```

Suggested edits to `CMakeLists.txt` (add the message generation bits):

Add `message_generation` to find_package catkin REQUIRED COMPONENTS, add the following sections:

```cmake
add_service_files(
  FILES
  Compute.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime rospy roscpp std_msgs geometry_msgs sensor_msgs tf tf2_ros
)

catkin_install_python(PROGRAMS
  scripts/compute_server.py
  scripts/compute_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

Suggested edits to `package.xml`: add

```xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

above the other dependency tags.

If you want, I can attempt to patch `CMakeLists.txt` and `package.xml` for you — let me know if it's okay for me to modify them here.

---

New service: `TaskOpt` (defined in `srv/TaskOpt.srv`)
- Request: string opt, int32 id
- Response: bool success, int32 status_code, string message

Quick run example (after building messages):

```bash
# start server
rosrun tsu200 taskopt_server.py

# call client
rosrun tsu200 taskopt_client.py my_opt 42
```

Notes:
- Ensure `srv/TaskOpt.srv` is listed via `add_service_files(...)` in `CMakeLists.txt` and `message_generation`/`message_runtime` deps are set in `package.xml`.
- After editing build files: `source /opt/ros/<distro>/setup.bash; catkin_make; source devel/setup.bash`.

---

New service: `TaskUpload` (defined in `srv/TaskUpload.srv`)
- Request: int32 id, geometry_msgs/Point HomePos, int32 PosNum, tsu200/Pos[] PosList
- Response: int32 id, bool success, int32 status_code, string message

Quick run example (after building messages):

```bash
# start TaskUpload server
rosrun tsu200 taskupload_server.py

# call client (example id 7)
rosrun tsu200 taskupload_client.py 7
```

Notes:
- Make sure to add `msg/Pos.msg` to `add_message_files(...)` and `srv/TaskUpload.srv` to `add_service_files(...)` in `CMakeLists.txt`, and include `geometry_msgs` as a dependency for message generation.
- Example CMake additions (merge with earlier suggestions):

```cmake
add_message_files(
  FILES
  Pos.msg
)

add_service_files(
  FILES
  Compute.srv
  TaskOpt.srv
  TaskUpload.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime rospy roscpp std_msgs geometry_msgs
)

catkin_install_python(PROGRAMS
  scripts/compute_server.py
  scripts/compute_client.py
  scripts/taskopt_server.py
  scripts/taskopt_client.py
  scripts/taskupload_server.py
  scripts/taskupload_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

After editing, build with `catkin_make` from your workspace root and `source devel/setup.bash` before running the nodes.
