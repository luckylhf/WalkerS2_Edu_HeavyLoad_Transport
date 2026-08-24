# Walker S2 双臂抱箱 Demo

## 项目当前提供的能力

本项目提供一个 ROS2 Humble 视觉感知 demo，用于已知尺寸箱体的双手夹抱目标生成。

当前流程为：

```text
双目相机 PointCloud2
        ↓
支撑平面检测
        ↓
箱体点云分割与尺寸匹配
        ↓
箱体 OBB 位姿估计
        ↓
TF 查询 base_link ← 点云 header.frame_id
（头部坐标链只是没有完整 TF 时的回退路径）
        ↓
发布左、右手预抓取/夹抱位姿
```

箱体尺寸来自 `config/box_grasp_ros2.yaml` 的型号库 `box_models`，由 `box_model`
选择当前型号（当前为 `small_box3`）：

```text
长度：400 mm
宽度：300 mm
高度：230 mm
```

`base_box` 保存完整默认参数，其余型号只写需要覆盖的字段，未写字段自动继承。

## 是否已经提供双手夹抱信息？

已经提供，但目前提供的是“双手夹抱目标位姿”，不是完整的双臂动作执行。

项目里有两个不同的姿态 GUI，不要混淆：

**1. 假点云仿真 GUI（`fake_box_scene_ros2.py`，`gui:=true` 时启动）**

```bash
ros2 launch box_grasp_demo box_grasp_sim.launch.py gui:=true ik:=true
```

可手动输入箱体参数、点击“随机箱子”，以及“回零姿态”“预抓取姿态”“抓取姿态”。
按钮通过 ROS 命令切换 IK 目标，数值 IK 求解每侧肩部到 `wrist_roll_link` 的 7 个
关节，并发布到 `/joint_states`。IK 的简化模型把 `waist_pitch_joint` 锁定在零位，
但节点会读真机 waist 读数做后仰补偿（把目标换算回锁腰模型下的等效位姿）。

“随机箱子”的取值范围是经 Pinocchio IK 验证的保守共同工作区：

```text
箱心 X   ：0.28--0.33 m
箱心 Y   ：-0.10--0.04 m
桌面高度 ：0.87--0.93 m
偏航角   ：82--98°（长边大致朝向机器人）
```

手动输入范围外的箱体时，IK 日志会报告位置/姿态残差。

**2. 姿态切换 GUI（`arm_pose_gui.py`，真机与录制帧仿真使用）**

只有 6 个模式按钮，没有箱体参数输入：预抓取 / 抓取 / 抓取后 / 收回 / 放置 / 零位，
分别向 `pose_command_topic`（默认 `/demo3/pose_command`）发布对应模式字符串。

读取相机点云并检测到箱体后，节点会发布：

- 箱体中心位姿和方向；
- 左手预抓取位姿；
- 右手预抓取位姿；
- 左手夹抱位姿；
- 右手夹抱位姿；
- 放置平台中心及箱体放置位姿；
- RViz 中的箱体和左右手接近箭头。

左右手约定：

```text
机器人坐标 +Y：左手
机器人坐标 -Y：右手
```

当前型号（`small_box3`）下：双手夹抱箱体的短边端面（`grasp_long_edge: false`），
接触点在箱顶下方 60 mm；`side_clearance: -0.02` 为负值，表示抓取点比箱体端面
再向内压入 20 mm；预抓取点由抓取点沿接近方向后退 80 mm。这些值都在
`box_models` 型号库里按型号配置：

```yaml
tool_contact_below_top: 0.06   # m，箱顶向下 60 mm
side_clearance: -0.02          # m，负值 = 朝箱体内部压入
pregrasp_distance: 0.08        # m，预抓取相对抓取点的后退距离
grasp_long_edge: false         # false = 抓短边
```

腕部到抓取（tool）坐标系的固定安装变换在 YAML 顶层配置，用 rpy + 平移表示：

```yaml
left_wrist_to_grasp_rpy: [...]              # rad
left_wrist_to_grasp_translation_xyz: [...]  # m
right_wrist_to_grasp_rpy: [...]
right_wrist_to_grasp_translation_xyz: [...]
```

变换约定为 `p_wrist = R_wrist_grasp * p_grasp + t_wrist_grasp`，节点按
`T_wrist = T_grasp @ inv(T_wrist_grasp)` 反算腕部位姿，同时发布抓取位姿和
换算后的 `wrist_roll_link` 位姿。

tool 坐标系的轴向约定（零位手腕下相对 `base_link` 的世界方向）。下面的数值是用
Pinocchio 对 `s2_v1.urdf` 做零位 FK、再乘上 `wrist_to_grasp` 实算出来的：

```text
        红 +X                绿 +Y            蓝 +Z
左手  [ 0,  0.174, -0.985]  [1, 0, 0]  [ 0, -0.985, -0.174]
右手  [ 0,  0.174,  0.985]  [1, 0, 0]  [ 0,  0.985, -0.174]

绿 +Y：机器人正前方（base_link +X），左右手一致，且是精确对齐
蓝 +Z：左手朝右（base_link -Y）、右手朝左（base_link +Y），指向箱体内部
红 +X：Y × Z（右手系；左手朝下、右手朝上）
```

注意 `±0.174 ≈ sin 10°`：除绿轴外，蓝轴和红轴在零位下都有约 10° 倾斜，
「朝右 / 朝上」是近似描述，不是精确轴对齐。

当前没有实现以下内容（已逐条核对代码）：

- MoveIt 双臂轨迹规划（仓库内无任何 moveit 依赖）；
- 手指/夹爪闭合命令（夹爪只做 RViz 可视化，无控制指令）；
- 六维力传感器接触检测；
- 起吊后的载荷稳定性判断。

**注意：手臂动作是会真实执行的。** `box_grasp_node_ros2.py` 本身只发布目标位姿
和关节角，但 `trigger_detect_box.py` / `navigate_box_workflow.py` 会通过
`/v1/motions` HTTP 接口驱动 `left_arm` / `right_arm` / `head`，且双臂（必要时连同
`biped`）在同一个请求里下发，属于同步执行。运行这两个脚本前请确认现场安全，并
准备好已验证的硬停/急停手段。

## 项目文件

```text
src/box_grasp_demo/
├── scripts/
│   ├── box_grasp_node_ros2.py       检测 + 抓取位姿主节点（DetectBox action）
│   ├── arm_ik_pinocchio.py          Pinocchio + CasADi 双臂 IK 节点
│   ├── arm_pose_gui.py              姿态模式切换 GUI（6 个按钮）
│   ├── trigger_detect_box.py        真机单机触发：检测 + IK + 手臂动作
│   ├── trigger_detect_box_sim.py    离线仿真触发（不访问真机 HTTP）
│   ├── navigate_box_workflow.py     导航 + 抓取 + 搬运 + 放置编排
│   ├── calibrate_table_height.py    桌面高度一键标定
│   ├── recorded_frame_publisher.py  重放 rosbag 提取的静态帧
│   ├── fake_box_scene_ros2.py       假点云桌面/箱体场景（自带 GUI）
│   ├── grasp_validation_ros2.py     双臂抓取位姿几何校验
│   ├── joint_state_bridge.py        /mc/joint_states → /joint_states
│   └── head_tf_bridge.py            头部 TF 桥接
├── src/box_grasp_demo/
│   ├── detector_open3d.py           点云检测和抓取几何计算
│   └── arm_ik/                      IK 求解核心与可达性扫描
├── config/
│   ├── box_grasp_ros2.yaml          参数配置（顶层 key 为 /** 通配符）
│   ├── box_grasp_demo.rviz          真机 / 录制帧 RViz 配置
│   └── box_grasp_sim.rviz           假点云仿真 RViz 配置
├── launch/
│   ├── box_grasp_demo.launch.py     真机启动文件
│   ├── box_grasp_recorded_frame.launch.py  录制帧离线仿真
│   └── box_grasp_sim.launch.py      假点云仿真
├── vendor/ubt_robot/                导航 SDK wheel（x86_64 / aarch64）
├── CMakeLists.txt / package.xml     ROS2 包信息
└── README.md                        本说明
```

`DetectBox.action` 与 `SuggestCrouch.srv`、`ComputePlaceIK.srv` 定义在独立包
`src/box_grasp_demo_msgs/`，代码统一 `from box_grasp_demo_msgs.action import ...`。

`src/box_grasp_demo/arm_ik/` 下的 `test_ik.py` 和 `test_ik_reachability.py` 不是
单元测试，而是可达性扫描脚本；`config/box_grasp_ros2.yaml` 里
`DESIRED_*_TABLE_Z`、`TABLE_HEIGHT_MIN/MAX` 等常量的数值依据就来自
`test_ik_reachability` 的扫描结果。

机器人模型（`walker_s2_description` 包）：

```text
src/walker_s2_description/urdf/s2_v1/s2_v1.urdf
src/walker_s2_description/meshes/
```

关键末端链路：

```text
左臂：L_shoulder_pitch_link → L_elbow_roll_link → L_wrist_roll_link
右臂：R_shoulder_pitch_link → R_elbow_roll_link → R_wrist_roll_link
```

## 相机和标定信息

相机话题来自需求文件：

```text
/sensor/camera/stereo/april_tag/results
/sensor/camera/stereo/color/info
/sensor/camera/stereo/color/raw
/sensor/camera/stereo/depth/info
/sensor/camera/stereo/depth/raw
/sensor/camera/stereo/pointcloud/raw
```

当前 demo 只使用点云，消息类型为 `sensor_msgs/PointCloud2`，默认输入话题为
`config/box_grasp_ros2.yaml` 中的：

```yaml
input_topic: /sensor/camera/stereo/pointcloud/raw
```

启动时可以覆盖，例如本地直接运行相机驱动：

```bash
ros2 launch box_grasp_demo box_grasp_demo.launch.py \
  input_topic:=/sensor/camera/stereo/pointcloud/raw
```

`color/raw`、`depth/raw` 和相应 `info` 没有参与箱体检测，可用于后续 2D-3D 融合
和调试。AprilTag 结果是后续姿态先验的预留接口，不是当前检测的必需条件。

### 相机外参：默认走 TF，不要套用标定文件

真机 `/tf_static` 已经包含点云 frame（`stereo_left_rectified_optical_frame`）到
`head_pitch_link` 的安装变换，节点直接查询
`base_link ← PointCloud2.header.frame_id` 即可完成坐标转换。

因此 `config/box_grasp_ros2.yaml` 中 **`camera_extrinsic_file` 必须保持为空**：

```yaml
camera_extrinsic_file: ""                       # 保持为空
camera_parent_frame: head_pitch_link            # 仅回退路径使用
camera_extrinsic_direction: parent_from_camera  # 仅回退路径使用
```

再指定一份标定文件会在 TF 之上叠加第二次变换，直接导致坐标错误。只有当真机
没有发布点云 frame 的完整 TF 链时，才需要提供标定文件走回退路径；若该文件的
变换方向相反，把 `camera_extrinsic_direction` 改为 `camera_from_parent`。

## 桌面和放置平台

桌面高度不再直接写成 `target_frame` 下的绝对区间，而是分成两步：

```yaml
ground_frame: base_footprint   # 地面参考 frame
table_height: 0.85             # 抓取桌面【离地绝对高度】，卷尺实测即可
support_height_min: -0.1       # 相对上面算出的桌面 Z 的偏移下限
support_height_max: 0.1        # 相对上面算出的桌面 Z 的偏移上限
```

节点先由 TF 动态读取 `base_footprint → base_link` 的 `translation.z` 得到
base_link 离地高度（真机下蹲/站立会变），再换算：

```text
桌面在 base_link 系的 Z = table_height - base_link离地高度
实际过滤区间 = [桌面Z + support_height_min, 桌面Z + support_height_max]
```

例如桌面离地 0.85、base_link 离地 0.866 时桌面 Z≈-0.016，过滤区间为
`[-0.116, 0.084]`。所以 `support_height_min/max` 是**相对偏移**，不是绝对高度，
改 `pointcloud_offset` 的 Z 时需要同步调整这两个值。

箱体夹抱关键点按以下几何计算（当前型号抓短边，`grasp_long_edge: false`）：

```text
箱体中心：C
箱体长边单位轴：u（双手沿 ±u 分居两侧），竖轴：n
箱体长度 L = 0.40 m，宽度 W = 0.30 m，高度 H = 0.23 m

接触高度   = H/2 - tool_contact_below_top          = 0.115 - 0.06 = 0.055
左抓取点   = C + u × (L/2 + side_clearance) + n × 接触高度
右抓取点   = C - u × (L/2 + side_clearance) + n × 接触高度
左/右预抓取点 = 对应抓取点沿接近方向后退 pregrasp_distance

其中 side_clearance = -0.02（负值 = 向箱体内压入 20 mm），
     pregrasp_distance = 0.08 m
```

`+Y` 方向一侧为左手，`-Y` 一侧为右手。

放置平台不能只靠“高度范围”与取箱子的桌面区分，因为两个平面可能高度相同。需要配置放置平台 ROI：

```yaml
placement_roi_min: [x_min, y_min, z_min]
placement_roi_max: [x_max, y_max, z_max]
placement_roi_enabled: true
```

节点会在这个 ROI 内重新拟合支撑平面，并计算：

```text
放置平台中心：P
箱体放置中心：P + 平面法向 × (H/2)
```

放置位姿发布在：

```text
/box_grasp_demo/placement_pose
```

如果没有配置并启用 `placement_roi_min/max`，节点仍可计算夹抱位姿，但不会声称已经识别出放置平台。

## ROS2 输出话题

节点名称为 `box_grasp_demo`，话题都是私有话题（`~/`）。launch 的 `ns` 参数默认
为 `/demo3`，因此真机和仿真下的完整话题名都带 `/demo3` 前缀：

```text
/demo3/box_grasp_demo/box_pose            箱体中心位姿
/demo3/box_grasp_demo/place_box_pose      箱体放置目标位姿
/demo3/box_grasp_demo/placement_pose      放置平台中心（需启用 ROI）

# 抓取（tool）坐标系位姿
/demo3/box_grasp_demo/{left,right}_pregrasp_pose
/demo3/box_grasp_demo/{left,right}_grasp_pose
/demo3/box_grasp_demo/{left,right}_aftergrasp_pose

# 换算后的 wrist_roll_link 位姿
/demo3/box_grasp_demo/{left,right}_wrist_pregrasp_pose
/demo3/box_grasp_demo/{left,right}_wrist_grasp_pose
/demo3/box_grasp_demo/{left,right}_wrist_aftergrasp_pose
/demo3/box_grasp_demo/{left,right}_wrist_retract_pose
/demo3/box_grasp_demo/{left,right}_wrist_place_pose

/demo3/box_grasp_demo/markers             RViz 标记
/demo3/box_grasp_demo/status              JSON 状态
```

Action 与 Service：

```text
/demo3/box_grasp_demo/detect_box       box_grasp_demo_msgs/action/DetectBox
/demo3/box_grasp_demo/suggest_crouch   box_grasp_demo_msgs/srv/SuggestCrouch
```

消息类型：

```text
*_pose：geometry_msgs/PoseStamped
markers：visualization_msgs/MarkerArray
status：std_msgs/String，内容为 JSON
```

`status` 成功时类似（注意字段名是 `box_center_m`）：

```json
{
  "state": "target_ready",
  "frame": "base_link",
  "box_center_m": [0.48, 0.0, 0.06],
  "dimensions_m": [0.4, 0.3, 0.23],
  "wrist_to_grasp": {"left": {"translation_m": [...]}, "right": {"translation_m": [...]}},
  "placement": {}
}
```

## ROS2 启动方式

同步代码到机器人（真机代码路径为 `/home/ubt/demo3`）：

```bash
rsync -avc --exclude '__pycache__/' --exclude '*.pyc' \
  src/box_grasp_demo/ \
  s2_3_ros:/home/ubt/demo3/src/box_grasp_demo/
```

编译。必须显式指定 `/usr/bin/python3`：导航 SDK 只支持 CPython 3.10，激活 conda
后会把 ROS action 消息误编译成其它 ABI。

```bash
source /opt/ros/humble/setup.bash
conda deactivate  # 如果当前提示符仍是 (base)
cd ~/work/s2_demo3
colcon build --packages-up-to box_grasp_demo --symlink-install \
  --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3 \
               -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

启动（默认同时打开 RViz2）：

```bash
ros2 launch box_grasp_demo box_grasp_demo.launch.py
```

launch 参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `ns` | `/demo3` | 节点命名空间，与真机控制器隔离 |
| `input_topic` | `/sensor/camera/stereo/pointcloud/raw` | 点云输入话题 |
| `camera_extrinsic_file` | `""` | 保持为空，见上一节 |
| `rviz` | `true` | 是否打开 RViz2 |
| `ik` | `true` | 是否启动 Pinocchio IK 节点 |
| `gui` | `false` | 是否启动姿态切换 GUI |
| `publish_arm_tf` | `false` | 是否发布手臂 TF |
| `so_path` | `""` | 预编译 IK 求解器 `.so` 路径 |

只启动感知节点：

```bash
ros2 launch box_grasp_demo box_grasp_demo.launch.py rviz:=false ik:=false
```

启动前确认点云和 TF：

```bash
ros2 topic type /sensor/camera/stereo/pointcloud/raw
ros2 run tf2_ros tf2_echo base_link head_pitch_link
```

还需要确认点云 `header.frame_id` 到 `base_link` 的 TF 链连通。如果断链，需要补充
相机 frame 的静态 TF，或按上一节走 `camera_extrinsic_file` 回退路径。

### 真机抓取-搬运-放置编排流程

`navigate_box_workflow.py` 是完整的导航 + 抓取 + 搬运 + 放置编排程序，
在启动主程序（detect_box action + IK）与真机运控后运行：

```bash
ros2 run box_grasp_demo navigate_box_workflow.py \
  --map-id lqx3 --pick-target box1 --place-target put1 \
  --address 192.168.11.2:51000
```

常用参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `--map-id` | lqx3 | 地图 ID |
| `--pick-target` | box1 | 地图取箱点 targetId |
| `--mid-target` | mid1 | 搬运中间点 targetId |
| `--place-target` | put1 | 地图放箱点 targetId |
| `--address` | 0.0.0.0:51000 | ubt_robot SDK 地址（机器人 CC 51000 端口） |
| `--action-name` | `/demo3/box_grasp_demo/detect_box` | DetectBox action 名 |
| `--debug` | 关闭 | 每个高层步骤确认；机械臂路点逐点确认并逐点发送 |
| `--connect-timeout` | 10 | SDK 连接超时秒数 |

非 debug 流程（自动执行，动作合并）：

```text
 1 激活 biped home（call_action，整个流程仅一次）
 2 SetMap (A000012) → 3 Relocation global (A000013) → 4 NavTo box1 (A000002)
 5 腰部后仰（抓取前，抓取/检测/搬运全程保持）
 6 准备拍照姿态 → 7 拍照前下蹲 → 8 检测箱子
 9 合并：抓取段（预抓取/抓取/抬起/收回）+站直（1 个请求）
10 NavTo mid1 (A000002) → 11 NavTo put1 (A000002)
12 放置前下蹲（独立步骤，放置 IK 需在下蹲后的 base_link 高度重算）
13 合并：放置段+站直（1 个请求）
14 腰部恢复（放置后） → 15 收起双臂
```

关键规则（与 `trigger_detect_box.py` 共用）：

- **下蹲量是绝对量**：相对站立位的位移，负值向下，站直为 0。
  biped 的 `/v1/motions` goals 第三元素直接填该绝对量。
- **biped home 只在流程开始执行一次**：`/v1/call_action
  {"action":"s2/move_biped_home"}` 激活站立功能，之后才能下蹲。流程中
  不再使用 `SetLegMode STAND`，导航到位后也不需要重复 home；动作完成后
  的站直用 motions 下蹲量 0 即可。
- **下蹲目标由主程序计算**：trigger 通过 suggest_crouch 服务获取建议
  下蹲量，期望桌面相对 base_link 的 Z 为主程序常量
  `DESIRED_GRASP/PLACE_TABLE_Z = 0.035`（不可配置，0.85m 桌下蹲约 5cm）。
- **放置下蹲必须独立于放置段**：放置 IK 依赖下蹲后的 base_link 高度，
  所以先下蹲、再重算并执行放置段，不能合并成一个请求。
- **motion 服务任务串行**：biped 与手臂共用 `/v1/motions` 任务队列，
  上一任务未结束时新命令返回 `Task is running`；程序自动等待重试
  （`MOTION_BUSY_RETRY_ATTEMPTS` × `MOTION_BUSY_RETRY_DELAY_SEC`），
  并在下蹲到位后额外等待 `CROUCH_SETTLE_SEC` 让任务收尾。
- **非 debug 合并发送**：下蹲+手臂段合并为单个请求（每路点 = biped 6 维
  + 双臂 14 维），`vel_scale` 用 `COMBINED_VEL_SCALE`（0.15）；debug
  模式保持逐步确认发送。

不导航的单机流程（无地图）用 `trigger_detect_box.py`：

```bash
ros2 run box_grasp_demo trigger_detect_box.py
```

相关环境变量（默认值取自 `trigger_detect_box.py`）：

| 环境变量 | 默认 | 说明 |
|---|---|---|
| `MOTION_BASE_URL` | `http://192.168.11.2:12300` | 运动 HTTP 服务地址 |
| `MOTION_VEL_SCALE` | 0.2 | 单步手臂动作速度 |
| `COMBINED_VEL_SCALE` | 0.15 | 合并请求速度 |
| `BIPED_HOME_SETTLE_SEC` | 3.0 | home 到位后收尾等待（秒） |
| `CROUCH_SETTLE_SEC` | 2.0 | 下蹲任务收尾等待（秒） |
| `MOTION_BUSY_RETRY_ATTEMPTS` | 5 | `Task is running` 重试次数 |
| `MOTION_BUSY_RETRY_DELAY_SEC` | 2.0 | 重试间隔（秒） |
| `WAIST_TILT_BACK_RAD` | 0.32 | 抓取前腰部后仰角（≈18.3°） |

### 抓取桌面高度（table_height）说明

`table_height` 是**抓取桌面离地绝对高度（m）**，在
`config/box_grasp_ros2.yaml` 中配置。抓取桌与放置桌相互独立，
两张桌子高度不同时分别配置。

**直接填写卷尺量出的桌面离地高度即可**，不需要标定流程。抓取时
箱子位置由视觉检测给出（base_link 系），`table_height` 只用于计算
下蹲量（让桌面落在 IK 可达区间）和桌面高度合法性校验，几厘米的
卷尺误差不会影响抓取。

**怎么标定（如需核对系统值）：**

```bash
ros2 run box_grasp_demo calibrate_table_height.py
```

脚本流程：把箱子放在抓取桌面上 → 输入卷尺实测桌面高度 → 自动检查
头部姿态（未低头会主动低头）→ 执行一次检测 → 读 TF 的 base_link
离地 → 按公式输出建议填写值：

```text
table_height = base_link离地 + 视觉实测桌面Z + 一点余量
```

**当前值：** 仓库示例与真机现场均为 `0.85`（换桌后按卷尺实测直接修改）。

**base_link 离地高度**可用下面的命令读取（`Translation.Z`，下蹲/站立
会变化），用于核对下蹲量计算：

```bash
ros2 run tf2_ros tf2_echo base_footprint base_link
```

**抓取下蹲量怎么算（主程序 suggest_crouch 服务）：**

```text
photo/grasp 下蹲量 = (table_height - DESIRED_GRASP_TABLE_Z) - 站立 base_link 高度
                  = (0.85 - 0.035) - 0.866 ≈ -0.051 m（约下蹲 5cm）
```

其中 `DESIRED_GRASP_TABLE_Z = 0.035` 与站立高度 `base_link_height = 0.866`
都是主程序常量，不可通过参数修改。

**合法范围：** `TABLE_HEIGHT_MIN/MAX = 0.75/1.10`（主程序常量）。桌面超出
该范围时主程序直接报错、拒绝执行检测/抓取/放置。

### 放置高度（place_table_height）说明

`place_table_height` 是**放置桌面离地绝对高度（m）**，在
`config/box_grasp_ros2.yaml` 中配置，与抓取桌 `table_height` 相互独立，
允许两张桌子高度不同。

**为什么不能直接填卷尺量出的桌子真实高度？**

放置时箱底高度 = `place_table_height - base_link离地高度 + place_clearance`。
而 base_link 离地高度来自 TF（`base_footprint→base_link.z`），视觉点云
也有相机标定/外参误差——这两套"系统坐标"与卷尺物理尺寸并不完全一致。
直接填卷尺值会把箱子往下压到桌子里。

**应该怎么填写？**

填"系统实测"的离地高度：

```text
place_table_height = base_link离地高度 + 视觉实测放置桌面Z + 一点余量
```

**推荐设置方案：一键标定脚本（抓取桌/放置桌通用）**

```bash
ros2 run box_grasp_demo calibrate_table_height.py
```

脚本流程：把箱子放在要标定的桌面上 → 输入卷尺实测桌面高度 →
**自动检查头部姿态（未低头会主动低头，保证相机能看到桌面）** →
自动执行一次检测 → 反推视觉实测桌面 Z（箱子中心 Z − 箱高/2）→
读 TF 的 base_link 离地 → 按公式输出建议填写值：

```text
系统桌面高度 = base_link离地 + 视觉实测桌面Z + 余量(≈0.005)
```

输出示例：

```text
卷尺实测桌面高度:           0.780 m
视觉实测桌面Z(base_link系): 0.060 m
base_link 离地高度:         0.815 m
推算桌面高度(建议填写值):   0.880 m
请将推算值填写到 config/box_grasp_ros2.yaml：
  抓取桌 → table_height:      0.880
  放置桌 → place_table_height: 0.780
```

**当前值：** `config/box_grasp_ros2.yaml` 中为 `place_table_height: 0.78`
（真机现场实测值）。更换桌子后需按上述步骤重新标定。

YAML 注释里另有一组标定示例：下蹲时 TF 离地 0.815 + 视觉桌面 Z≈0.060 ≈ 0.875，
取 0.88，配合 `place_clearance=0.005` 得到 place_z≈0.070，略高于视觉实测桌面。
那只是演示公式怎么用，不是当前生效值。

**放置下蹲量怎么算（主程序 suggest_crouch 服务）：**

```text
place 下蹲量 = (place_table_height - DESIRED_PLACE_TABLE_Z) - 站立 base_link 高度
            = (0.78 - 0.035) - 0.866 ≈ -0.121 m（约下蹲 12cm）
```

其中 `DESIRED_PLACE_TABLE_Z = 0.035`（期望桌面相对 base_link 的 Z）与
站立高度 `base_link_height = 0.866` 都是主程序常量，不可通过参数修改。

相关参数：`place_clearance`（放置离桌安全间隙，默认 0.005）、
`base_link_height`（仅 TF 查询失败时的回退值，站立实测 0.866）、
`place_x/place_y/place_yaw_deg`（放置位姿，yaw 需 ±70°~±90° 才能收敛）。

## 运行前检查

项目已提供 [config/box_grasp_demo.rviz](config/box_grasp_demo.rviz)，会显示以下内容：

1. `/sensor/camera/stereo/pointcloud/raw`；
2. `base_link` 和 `head_pitch_link` TF；
3. `/demo3/box_grasp_demo/markers`；
4. Walker S2 URDF 模型；
5. `/demo3/box_grasp_demo/left_grasp_pose` 和 `.../right_grasp_pose`；
6. `/demo3/box_grasp_demo/placement_pose`。

RViz 按 X=红、Y=绿、Z=蓝 显示坐标轴。抓取（tool）坐标系在零位手腕下的实际指向为：

```text
绿轴 +Y：机器人正前方（base_link +X），左右手一致，精确对齐
蓝轴 +Z：左手朝右（base_link -Y）、右手朝左（base_link +Y），即指向箱体内部
红轴 +X：Y × Z（右手系；左手朝下、右手朝上）
```

蓝轴和红轴在零位下有约 10° 倾斜，详见「是否已经提供双手夹抱信息？」一节的实算值。

确认箱体中心、长短轴、左右箭头均正确后，才能接入 MoveIt 或机器人控制节点。

## 无机器人 RViz 仿真

当前可以在没有连接机器人的情况下验证“点云识别箱子 → 计算双臂目标位姿”的完整链路。启动：

```bash
source /opt/ros/humble/setup.bash
cd ~/work/s2_demo3
colcon build --packages-up-to box_grasp_demo --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 launch box_grasp_demo box_grasp_sim.launch.py
```

仿真节点把假点云发布到 `<ns>/pointcloud`（`ns` 默认 `/demo3`，即
`/demo3/pointcloud`），内容为 `sim_world` 坐标系下的抬高桌面和一个
`400 x 300 x 200 mm` 的箱子。`fake_box_scene_ros2.py` 的默认值为：

```text
table_z      0.90 m
box_center   [0.65, 0.00, 1.00] m
box_yaw_deg  90.0°
```

Walker S2 的 `base_link` 通过静态 TF 放在 `z=0.904 m`（launch 参数
`robot_base_z`），使零位姿态下双脚最低点接触 `sim_world/z=0` 地面。检测节点从该
点云拟合平面、分割箱体并发布双臂位姿。RViz 会显示：

- `/demo3/pointcloud` 假点云；
- Walker S2 URDF 模型（默认关节零位姿）；
- 绿色箱体 Marker；
- 左右抓取坐标轴和接近箭头；
- 箱体中心与抓取 Pose。

仿真默认不显示橙色放置 Marker，因为没有配置目标放置平台 ROI。只有设置
`placement_roi_enabled: true` 以及对应的 `placement_roi_min/max` 后，橙色才表示
箱子放置目标；它不是箱子底面。

终端应看到 `dual-arm grasp validation: {"state":"PASS", ...}`。也可以单独查看：

```bash
ros2 topic echo /demo3/box_grasp_demo/validation --once
ros2 topic echo /demo3/box_grasp_demo/status --once
```

注意该 launch 只提供假点云链路，没有 `/demo3/sim/base_link_height_cmd` 等仿真
下蹲接口，因此不能在这个模式下跑 `trigger_detect_box_sim.py`；需要验证下蹲和
完整姿态序列时，改用 `box_grasp_recorded_frame.launch.py` 的录制帧仿真。

这个仿真验证的是感知和几何目标生成，不代表机器人一定能到达该位姿；真实验证还需要 URDF/当前关节 TF、双臂 IK、碰撞检查、夹爪尺寸和控制器。

Walker S2 的 URDF 由 `walker_s2_description` 包安装到：

```text
share/walker_s2_description/urdf/s2_v1/s2_v1.urdf
share/walker_s2_description/meshes/
```

仿真启动会自动发布所有可动关节的零位状态，并由 `robot_state_publisher`
生成完整身体、双臂和双手的 TF。
