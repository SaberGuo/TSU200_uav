# ROS Proxy 地面交互消息

## 一、系统总体功能

`ros_proxy` 模块用于在多机 ROS 网络中建立**地面站与无人机间的消息代理通信通道**。  
其核心功能包括：

- 使用 **Nanomsg (NN_BUS)** 实现以太网跨主机通信；
    
- 支持 ROS 消息的**序列化、反序列化、转发与同步**；
    
- 管理多架无人机间的消息交换；
    
- 在地面端（commander 模式）集中显示与控制多机状态；
    
- 在飞行端（drone 模式）执行控制与反馈传递。
    

---

## 二、主要消息类型定义

枚举类型 `MESSAGE_TYPE` 定义如下：

|类型名称|枚举值|含义|
|---|---|---|
|`ODOM`|100|位姿（里程计）消息|
|`ONE_TRAJ`|101|轨迹消息|
|`POINT_CLOUD`|102|点云数据|
|`GOAL`|103|目标位姿|
|`INIT_POSE`|104|初始化位姿|
|`CONTROL`|105|控制命令|
|`SWARM_MATRIX`|106|群体编队矩阵信息|
- 自定义服务 `TaskUpload.srv`：请求包含任务 ID、Home Pos、PosNum、PosList 等字段。
    
- 自定义消息 `Pos.msg`：定义单个位置点类型。
---

## 三、通信与消息格式

所有传输消息均采用以下二进制格式：

```
`[MESSAGE_TYPE][FROM_ID][MSG_SIZE][ROS_SERIALIZED_MSG]`
```

- `MESSAGE_TYPE`：消息类型；
    
- `FROM_ID`：发送端无人机 ID；
    
- `MSG_SIZE`：ROS 原始消息长度；
    
- `ROS_SERIALIZED_MSG`：通过 `ros::serialization` 序列化后的内容。
    

### 序列化与反序列化函数

- `serializeTopic(MESSAGE_TYPE, msg, buf)`
    
- `deserializeTopic(buf, msg)`
    

---

## 四、地面端（Commander）消息交互

地面端以集中控制与可视化为主，主要话题交互如下：

| ROS Topic/service                     | 类型                             | 方向  | 功能描述          |
| ------------------------------------- | ------------------------------ | --- | ------------- |
| `/initpose`                           | `controller_msgs::PoseStamped` | sub | 接收无人机初始位姿     |
| `/control`                            | `controller_msgs::control`     | sub | 接收控制指令并转发     |
| `/swarm_matrix`                       | `controller_msgs::SwarmMatrix` | sub | 接收群体矩阵信息      |
| `/move_base_simple/goal`              | `geometry_msgs::PoseStamped`   | sub | 接收群体目标        |
| `/drone_X_mavros/local_position/odom` | `nav_msgs::Odometry`           | pub | 发布各机体的里程计数据   |
| `/drone_X_planner/trajectory`         | `visualization_msgs::Marker`   | pub | 可视化轨迹         |
| `/drone_X_planner/point_cloud`        | `sensor_msgs::PointCloud2`     | pub | 可视化点云数据       |
| `/task_upload`                        | `tsu200::TaskUpload`       | srv | 地面站向无人机发起任务上传 |
| `/task_opt`                           |  `tsu200::TaskOpt`         | srv | 地面站向无人机发起任务操作|
| `/task_download`                      |  `tsu200::TaskDownload`    | srv | 地面站向无人机发起任务下载|
其中 `X` 为无人机编号。

### 4.1 /task_upload
服务为单次发送请求。
* 消息内容
```
int32 id
geometry_msgs/Point HomePos  #起飞与返回点
int32 PosNum                 #航点数量
tsu200/Pos[] PosList         #航点列表
---
int32 id                     #任务ID
bool success                 #执行成功情况
int32 status_code            #错误状态码，成功为0，其余为错误状态
string message               #返回信息内容
```
其中航点信息为tsu200/Pos类型，具体如下
```
float64 x
float64 y
float64 z
string task_type    #航点处需要执行的任务类型 1、代表云台向前拍摄；2、代表云台向左拍摄；3、代表云台向右拍摄；4、代表RFID收数据
string info
```
### 4.2 /task_opt
服务为单次发送请求。
* 消息内容
```
int32 opt                     #任务操作码，1、开始任务；2、暂停任务；3、停止任务；
int32 id                      #任务ID
---
bool success                  #执行成功情况
int32 status_code             #错误状态码，成功为0，其余为错误状态
string message                #返回信息内容
```
### 4.3 /task_download
服务为单次发送请求。
* 消息内容
```
int32 id                      #任务ID
---
int32 id                      #任务ID
geometry_msgs/Point HomePos   #起飞与返回点
int32 PosNum                  #航点数量
tsu200/Pos[] PosList          #航点列表
``` 

---

## 五、飞行端（Drone）消息交互

无人机端负责接收命令并发布状态信息。其主要 ROS 话题包括：

| ROS Topic                      | 类型                             | 方向      | 功能描述            |
| ------------------------------ | ------------------------------ | ------- | --------------- |
| `my_odom`                      | `nav_msgs::Odometry`           | sub     | 自身里程计数据（上报）     |
| `/others_odom`                 | `nav_msgs::Odometry`           | pub     | 接收其他无人机位姿       |
| `/goal`                        | `geometry_msgs::PoseStamped`   | pub/sub | 接收目标位置或广播目标     |
| `/initpose`                    | `geometry_msgs::PoseStamped`   | pub     | 发布初始化位姿         |
| `/control`                     | `controller_msgs::control`     | pub/sub | 控制消息交互          |
| `/swarm_matrix`                | `controller_msgs::SwarmMatrix` | pub     | 发布群体矩阵          |
| `/point_cloud`                 | `sensor_msgs::PointCloud2`     | sub     | 点云同步传输          |
| `/broadcast_traj_from_planner` | `controller_msgs::MinTraj`     | sub     | 路径规划轨迹          |
| `/broadcast_traj_to_planner`   | `controller_msgs::MinTraj`     | pub     | 广播接收的轨迹         |
| `/task_upload_server`          | `tsu200_uav::TaskUpload`<br>   | srv     | 无人机端处理地面站任务上传请求 |

---

## 六、扩展功能（Drone 版本新增）

在 `ros_proxy_drone.cpp` 中增加了**地面指令联动执行功能**，由话题 `/control` 触发：

|控制码|功能说明|执行动作|
|---|---|---|
|154|PX4 起飞|执行 `rostopic pub -1 /px4ctrl/takeoff_land ... takeoff_land_cmd: 1`|
|155|PX4 降落|执行 `rostopic pub -1 /px4ctrl/takeoff_land ... takeoff_land_cmd: 2`|
|156|PX4 起始移动|自动生成 `/move_base_simple/goal` 位姿消息，移动 +4 米并起飞至 1m|

对应回调函数为：

`void drone_callback(const controller_msgs::controlPtr &msg)`

此外，代码中保留了被注释的 `track.sh`, `mapping.sh` 等脚本接口，用于未来拓展自主建图、目标跟踪等功能。
TSU200_uav 任务模块
---
补充如下功能：
- 地面站可通过服务 `TaskUpload` 向无人机发送一系列位置点（Home Pos + PosList）以执行任务。
    
- 无人机端接收到服务调用后，可解析该任务并进入执行模式。
    
- 此机制可与原有 CONTROL／GOAL 通信机制并行，实现“任务级”交互。
## 七、系统流程概要

1. 地面站初始化
    
2. 控制命令：地面站将 CONTROL 指令下发至各无人机（如起飞、降落、移动）。
    
3. 状态上报：无人机通过 `/drone_X_mavros/local_position/odom` 向地面站发布里程计。
    
4. 任务上传（扩展）：地面站调用 `TaskUpload` 服务发送任务点列表 → 无人机接收并执行。

5. 任务执行：地面站调用`TaskOpt(1)`服务发送任务开始命令，参数携带`TaskID`无人机开始执行任务。

6. 任务暂定：地面站调用`TaskOpt(2)`服务发送任务开始命令，参数携带`TaskID`无人机开始执行任务。

7. 任务停止：地面站调用`TaskOpt(3)`服务发送任务开始命令，参数携带`TaskID`无人机开始执行任务。
    
8. 轨迹 & 点云可视化：地面站订阅轨迹`/drone_X_mavros/local_position/odom`、预测轨迹`/drone_X_planner/trajectory`与点云话题`/drone_X_planner/point_cloud`用于状态监控。

9. 当电池电量低于18V时，无人机自动任务暂定，并返回`HomePos`,地面站判断到达后，发送命令`/px4ctrl/takeoff_land takeoff_land_cmd: 2`,降落后启动机库回收充电程序；

10. 充电完成后，如有任务处于暂停状态，发送起飞命令`/px4ctrl/takeoff_land takeoff_land_cmd: 1`，地面站调用`TaskOpt(1)`服务发送任务开始命令，参数携带`TaskID`无人机开始执行任务。

11. 无人机执行完成任务后，返回`HomePos`，地面站判断到达后，发送命令`/px4ctrl/takeoff_land takeoff_land_cmd: 2`,降落后启动机库回收充电程序;
## 八、总结

- 两个版本共用的基础功能包括：
    
    - ROS → Nanomsg → ROS 的双向消息桥；
        
    - 同步多机的 ODOM、TRAJ、GOAL、CONTROL、SWARM 数据；
        
    - commander 与 drone 模式的自动区分。
        
- `ros_proxy_drone.cpp` 在基础通信之上增加了对 PX4 控制命令的地面交互逻辑，实现了从地面直接控制无人机执行动作的接口。