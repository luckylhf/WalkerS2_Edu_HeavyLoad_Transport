# Walker S2 Edu 探索者 料箱重载自主搬运示例程序

本项目依托Walker S2 Edu探索者双足人形机器人（以下简称Walker S2），结合SDK开发套件与传统运动控制算法，实现料箱重物自主搬运全流程场景作业，展现机器人的环境感知、运动解算、重载行走及精准操作能力，具体作业流程及核心技术亮点如下：
1. 自主导航抵达作业区域：机器人依托vSlam视觉定位建图能力，自主规划行走路径，移动到作业区周边后，使用腰部RGBD识别定位二维码，精准移动至目标作业位，完成作业前就位，体现机器人环境感知与自主导航运动能力。
2. 视觉定位识别+双臂协同抓取抬升：通过双目相机采集点云数据，对已知尺寸的桌面料箱进行精准检测识别，自主标定机械臂接触、抓取点位，规划左右手预抓取、抓取、抬升、放置全流程位姿。同时依托Pinocchio+CasADi算法，完成14个手臂关节的逆运动学（IK）求解，实现双臂协同稳定抱取、抬升料箱，展现机器人视觉感知与多关节协同操控的性能。
3. 重载稳定行走+精准放置物料：机器人搭载重载物料状态下，依托重载运动控制技术结合vSlam实时定位能力，保持机身平稳行走，移动到卸料区周边后，使用腰部RGBD识别定位二维码，精准移动抵达指定卸料位，完成料箱平稳放置，验证了机器人重载工况下的行走稳定性与末端作业精准性。

本场景完整展示了Walker S2机器人 vSlam、双臂协同重载搬运、负载稳定行走、视觉感知 四大核心能力，充分适配仓储转运、工业物料搬运等实操场景，体现其在工业、仓储物流领域的落地应用价值与实操优势。

- 机器人 SSH 别名为：`s2_3_ros`。SSH 登录目标机器人，修改如下文件，密码请向您的技术支持人员获取。
    ```bash
    # ~/.ssh/config 文件添加 下面片段 
    Host s2_3_ros
      HostName 192.168.11.2
      User ubt
      Port 2222
    ```
- 机器人本体端代码路径：`/home/ubt/demo3`，此仓库代码都属于这部分
- 外部Ubuntu端代码路径：`~/work/s2_demo3`

---

## 目录结构

```text
demo3/
├── doc/                         使用文档、SOP、图片与许可证
│   └── LICENSES/                许可证范围、正文及第三方声明
│   └── Actuator/                二指夹爪配套夹抱配件3D打印文件
│   └── Demos/                   示例gif及交互物
├── src/
│   ├── box_grasp_demo/          感知、IK、导航与 SDK 集成 ROS2 包
│   │   ├── scripts/             ROS2 节点脚本
│   │   ├── src/box_grasp_demo/  检测器与 IK 核心
│   │   ├── config/              YAML 参数与 RViz 配置
│   │   ├── launch/              launch 文件
│   │   └── vendor/ubt_robot/    导航 SDK wheel
│   ├── box_grasp_demo_msgs/     DetectBox action 与相关 service
│   ├── shm_msgs/                共享内存消息定义
│   └── walker_s2_description/   机器人 URDF、mesh、配置与 launch
├── tools/
│   ├── motion_http_proxy/       运动 HTTP 代理及调用脚本
│   ├── run_offline_sim.sh       本地离线仿真启动脚本
│   ├── download_current_bag.sh        一键下载真机 rosbag
│   ├── record_current_bag_remote.sh   真机端录制脚本（本地副本）
│   ├── extract_current_bag.py         从 bag 提取静态点云帧
│   ├── save_img.sh                    真机下载 RGB 图片脚本（本地副本）
│   ├── robot_run.sh                   真机端 run.sh（本地副本）
├── robot_data/
│   ├── tf_analysis_current/           下载下来的 rosbag
│   └── current_bag/                   从 bag 提取出的单帧（点云/TF/关节）
├── .gitignore
└── README.md
```

---

## 许可证

本项目包含 Apache-2.0 代码、UBTECH 专有资产及第三方组件。各类文件的
许可范围、完整许可证正文和第三方声明见
[doc/LICENSES/README.md](doc/LICENSES/README.md)。

---

## 环境与依赖安装

本项目把依赖分成两套 Python 环境，不能混用：

- 系统 Python `/usr/bin/python3`：运行 ROS2 节点和 Open3D 检测器。
- conda 环境 `ik`：运行 Pinocchio + CasADi 双臂 IK。

### 1. ROS 2 Humble

建议使用 Ubuntu 22.04 + ROS 2 Humble，并安装构建工具：

```bash
sudo apt update
sudo apt install -y \
  python3-pip \
  python3-colcon-common-extensions \
  python3-rosdep
```

### 2. 系统 Python 依赖（检测节点）

检测节点 `box_grasp_node_ros2.py` 使用 `/usr/bin/python3`，需要 NumPy 和 Open3D：

```bash
sudo apt update
sudo apt install -y python3-numpy python3-yaml python3-websocket

# Open3D 0.19.x
python3 -m pip install open3d
```

如果系统 Python 启用了 PEP 668（部分较新发行版），改用：

```bash
python3 -m pip install --break-system-packages open3d
```

当前本地系统 Python 已安装：

```text
Python 3.10.12
numpy 1.26.4
open3d 0.19.0
```

### 3. conda IK 环境

IK 节点 `arm_ik_pinocchio.py` 运行在 conda 环境 `ik`，需要 Pinocchio 和 CasADi。

先安装 Miniconda，然后创建环境：

```bash
conda create -n ik python=3.10 -y
conda activate ik
conda install -c conda-forge pinocchio casadi numpy -y
```

如果希望 IK 环境也能本地调试 Open3D 检测器，可一并安装：

```bash
conda install -c conda-forge open3d -y
```

当前本地 `ik` 环境实测：

```text
Python 3.10.19
numpy 1.26.4
pinocchio 3.6.0
casadi 3.7.0
open3d 0.19.0
```

### 4. ROS 包依赖

安装本工作区 package.xml 声明的 ROS 依赖：

```bash
cd ~/work/s2_demo3
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

然后构建：

```bash
colcon build --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

本地离线仿真脚本 `tools/run_offline_sim.sh` 内部已经处理了 ROS 与 IK 两套
Python 的切换，因此日常运行不需要手动重复这些命令。

### 5. 验证依赖

```bash
# 系统 Python（检测器）
python3 -c "import numpy, open3d; print('open3d', open3d.__version__)"

# IK conda 环境
conda run -n ik python -c "import numpy, pinocchio, casadi; print('ik env ok')"
```

---

## 一、从机器人下载 RGB 图片

机器人端脚本订阅 `/sensor/camera/stereo/color/raw`，保存当前一帧为 PNG。
脚本在机器人上：`~/demo3/tools/save_img.sh`，本地副本：`tools/save_img.sh`。

### 1. 在机器人上抓一张图

```bash
ssh s2_3_ros ~/demo3/tools/save_img.sh
```

脚本会等待 `/sensor/camera/stereo/color/raw` 的一帧，保存到机器人
`~/demo3/images/`，文件名类似：

```text
stereo_color_20260812_181134.png
```

终端会打印实际保存的文件名。

### 2. 下载到本地

```bash
mkdir -p img
scp 's2_3_ros:/home/ubt/demo3/images/stereo_color_*.png' img/
```

如果只想下载刚生成的那一张，把 `stereo_color_*` 换成终端打印的具体文件名。

---

## 二、从机器人下载 rosbag

录制的是静止场景的一小段 bag，包含点云、TF 和关节状态：

```text
/tf_static
/tf
/sensor/camera/stereo/pointcloud/raw
/mc/joint_states
```

机器人端录制脚本：`/home/ubt/demo3/tools/record_current_bag_remote.sh`，本地副本：
`tools/record_current_bag_remote.sh`。

### 方式 A：一键录制并下载（推荐）本地执行

```bash
./tools/download_current_bag.sh 15
```

参数 `15` 是录制秒数，默认 15 秒。脚本会：

1. SSH 到机器人执行 `/home/ubt/demo3/tools/record_current_bag_remote.sh /tmp/tf_analysis_current 15`
2. 下载 `tf_analysis_current_0.db3` 和 `metadata.yaml`
3. 保存到本地 `robot_data/tf_analysis_current/`

### 方式 B：手动分步执行

```bash
# 1. 在机器人上录制
ssh s2_3_ros /home/ubt/demo3/tools/record_current_bag_remote.sh /tmp/tf_analysis_current 15

# 2. 下载到本地
mkdir -p robot_data/tf_analysis_current
scp 's2_3_ros:/tmp/tf_analysis_current/tf_analysis_current_0.db3' \
    robot_data/tf_analysis_current/
scp 's2_3_ros:/tmp/tf_analysis_current/metadata.yaml' \
    robot_data/tf_analysis_current/
```

录制脚本默认会清空并重建输出目录，所以同名 bag 会被覆盖，下载前请确认已保存好旧数据。

---

## 三、本地离线仿真

本地仿真不依赖真机 DDS，而是从已下载的 rosbag 中提取中间一帧静态点云、TF 和
关节状态，然后重放这一帧，启动检测 + IK + RViz。

### 前置条件

- 已安装 ROS 2 Humble
- 已安装 conda 环境 `ik`，其中包含 Pinocchio / CasADi
- 已按上文下载 rosbag 到 `robot_data/tf_analysis_current/`

### 启动

```bash
cd ~/work/s2_demo3
source /opt/ros/humble/setup.bash
./tools/run_offline_sim.sh
```

`run_offline_sim.sh` 会依次：

1. 运行 `tools/extract_current_bag.py`，从
   `robot_data/tf_analysis_current/tf_analysis_current_0.db3`
   提取一帧点云、TF 和关节状态到 `robot_data/current_bag/`
2. 使用 ROS 自带 Python 执行 `colcon build --symlink-install`
3. 若需要，生成或复用 IK 求解器 `.so`
4. 启动 `box_grasp_recorded_frame.launch.py`，打开 RViz

### 不启动 IK

只想看检测结果、不需要 IK 时：

```bash
source /opt/ros/humble/setup.bash
./tools/run_offline_sim.sh false
```

### 查看结果

RViz 会显示：

- 重放的真机点云
- Walker S2 模型
- 绿色箱体 Marker（视觉检测箱）
- 紫色半透明箱体 Marker（姿态 GUI 当前选中的目标箱；旁边带有阶段文字）
- 左右手抓取位姿和接近箭头

也可以在终端或新终端里查看话题：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /demo3/box_grasp_demo/status --once
ros2 topic echo /demo3/joint_states --once
```

离线仿真不要使用会连接真机运动 HTTP 的 `trigger_detect_box.py`。使用专用触发器可先
模拟下蹲，再执行 DetectBox 与 IK：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run box_grasp_demo trigger_detect_box_sim.py
```

确认后，触发器从 `suggest_crouch` 获取站立和目标高度，通过纯 ROS 仿真话题先回站立位，
再按 `0.05 m/s` 改变 `base_link` 高度。回放点云会同步改变相对高度，高度到位后才发送
DetectBox goal；RViz 以 `base_footprint` 为固定坐标系显示机身下降。检测和 IK 完成后，
姿态 GUI 可依次切换预抓取、抓取、抓取后、收回和放置；其中“收回”使用 `ik_data` 第 4 组，
箱体中心目标姿态为 `rx=ry=0, rz=-90°`，使双手继续位于箱体左右两侧。
此流程不会访问
真机 HTTP。可用 `--yes` 自动确认，或用
`--skip-crouch` 保持当前仿真高度直接检测。

仿真下蹲接口为：

- `/demo3/sim/base_link_height_cmd` (`std_msgs/msg/Float64`)：目标绝对离地高度，单位 m
- `/demo3/sim/base_link_height` (`std_msgs/msg/Float64`)：当前仿真离地高度，单位 m

录制帧没有腿部完整运动轨迹，因此检测几何、点云和 `base_link` 高度会正确变化，但腿部
关节不会显示真实的屈膝过程。

---

## 四、无机器人仿真方式说明

无机器人时有两种仿真方式，使用的数据和可验证的流程不同。

### 方式 A：离线真机点云完整仿真（推荐）

该方式使用 `robot_data/tf_analysis_current/` 中 rosbag 提取的一帧真机点云，能够验证
下蹲建议、箱体检测、双臂 IK，以及预抓取 / 抓取 / 抓取后 / 收回 / 放置姿态。
它不连接机器人，也不会调用真机 HTTP 运动接口。

终端 1 启动离线环境，并保持运行：

```bash
cd ~/work/s2_demo3
source /opt/ros/humble/setup.bash
./tools/run_offline_sim.sh
```

等待 RViz、检测节点、IK 节点和姿态 GUI 全部启动后，在终端 2 触发仿真流程：

```bash
cd ~/work/s2_demo3
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run box_grasp_demo trigger_detect_box_sim.py
```

触发器会先请求 `photo` 模式的下蹲建议。输入 `Y`（或直接回车）后，仿真机器人先回到
站立高度，再下蹲到建议高度；到位后发送 DetectBox goal，并打印箱体中心、尺寸、下蹲
建议和 `ik_data`。计算完成后，可在姿态 GUI 中检查以下 5 组姿态：

1. 预抓取
2. 抓取
3. 抓取后（抬起）
4. 收回：箱体中心 `rx=ry=0, rz=-90°`
5. 放置

GUI 中的箱体颜色含义为：绿色是检测得到的原始箱体，紫色是当前动作阶段的虚拟目标箱。
切换到“抓取后”时紫色箱会向上移动，切换到“收回”时会移动到收回目标并旋转为
`rz=-90°`，切换到“放置”时会移动到配置的放置位置；切换到“零位”会清除紫色目标箱。

常用触发参数：

```bash
# 不询问，自动执行仿真下蹲
ros2 run box_grasp_demo trigger_detect_box_sim.py --yes

# 保持当前仿真高度，直接检测
ros2 run box_grasp_demo trigger_detect_box_sim.py --skip-crouch

# 分别检查 photo / grasp / place 模式的下蹲建议
ros2 run box_grasp_demo trigger_detect_box_sim.py --crouch-mode photo
ros2 run box_grasp_demo trigger_detect_box_sim.py --crouch-mode grasp
ros2 run box_grasp_demo trigger_detect_box_sim.py --crouch-mode place
```

注意：虽然整个过程不连接机器人，但 `run_offline_sim.sh` 使用的是录制的真机点云，
因此它不是“假点云仿真”，并且运行前必须已有
`robot_data/tf_analysis_current/tf_analysis_current_0.db3`。

### 方式 B：纯假点云基础仿真

没有 rosbag 时，可以由程序生成桌面和箱子假点云，验证基础的“点云识别 → 抓取位姿”
链路：

```bash
cd ~/work/s2_demo3
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 launch box_grasp_demo box_grasp_sim.launch.py
```

该 launch 会发布一个放在桌面上的假箱子点云，并启动检测、抓取校验和 RViz。当前纯假
点云 launch 没有提供 `/demo3/sim/base_link_height_cmd` 和
`/demo3/sim/base_link_height` 仿真下蹲接口，所以不要在该模式下运行
`trigger_detect_box_sim.py`；需要验证下蹲和完整的 5 组 IK 姿态时，请使用方式 A。

---

## 五、真机运行

真机代码在 `/home/ubt/demo3`。真机 `tools/robot_run.sh` 会编译并启动检测 + IK，
**不会驱动机器人动作**，只发布目标位姿和关节角。本地副本：`tools/robot_run.sh`。

```bash
ssh s2_3_ros ~/demo3/tools/robot_run.sh
```

真机 launch 使用命名空间 `/demo3`，与真机控制器隔离。

查看检测状态：

```bash
ssh s2_3_ros 'source /opt/ros/humble/setup.bash; source ~/demo3/install/setup.bash; ros2 topic echo /demo3/box_grasp_demo/status --once'
```

### 为什么节点要跑在机器人本体

检测、IK 和编排节点全部在机器人本体运行，本地不通过 DDS 连接真机：

- SSH 别名 `s2_3_ros`（`192.168.11.2:2222`）只是登录通道，不是 ROS 2 DDS
  通道。在本地直接执行 `ros2 topic list` 看不到真机话题，必须像上面那样在
  SSH 内 source 后执行，且输出只存在于远程。
- 真机 launch 会无条件启动 `joint_state_bridge.py`，把 `/mc/joint_states`
  的关节名转换后发布到全局 `/joint_states`（含踝关节串并转换），供
  `robot_state_publisher` 按 URDF 生成完整 TF。该桥接节点不进 `/demo3`
  命名空间。

本地没有真机时请使用第三、四节的离线仿真，不要试图把真机话题桥接到本地。

### 地图导航抓放流程

`navigate_box_workflow.py` 是独立的导航编排程序。它复用
`trigger_detect_box.py` 的检测、IK 和机械臂动作实现，流程为：

```text
设置站立模式 → 设置地图 lqx3 → 全局重定位 → 导航到 box1
→ 检测并抓起箱子 → 恢复导航高度 → 导航到 put1 → 放置箱子
```

程序依赖 Walker S2 的 `ubt_robot` Python 3.10 SDK。项目已在
`src/box_grasp_demo/vendor/ubt_robot/` 附带 x86_64 和 aarch64 wheel；
`colcon build` 会检查 Python/CPU 架构并把匹配的 wheel 自动安装到工作区，
无需手动执行 `pip install`，也不会污染系统 Python。系统动态库仍需安装：

```bash
sudo apt install -y libmosquitto1
source /opt/ros/humble/setup.bash
colcon build --packages-up-to box_grasp_demo --symlink-install \
  --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3 \
               -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

显式指定 `/usr/bin/python3` 可避免激活 conda 后把 ROS action 消息误编译为
Python 3.12 ABI；导航 SDK 只支持 CPython 3.10。

先启动 `box_grasp_demo` 检测和 IK 节点，再运行自动流程：

```bash
source /opt/ros/humble/setup.bash
source ~/demo3/install/setup.bash
ros2 run box_grasp_demo navigate_box_workflow.py \
  --map-id lqx3 --pick-target box1 --place-target put1
```

真机首次联调使用 debug 模式；每个导航/抓放阶段及机械臂路点都会等待人工确认：

```bash
ros2 run box_grasp_demo navigate_box_workflow.py --debug
```

可用 `--address 192.168.11.2` 指定 SDK 服务地址。按 `Ctrl+C` 时程序会请求停止当前
UBT 导航技能并退出本地编排；它不能撤回已经由 HTTP 服务接收的机械臂或 biped 动作。
危险情况下必须使用现场已验证的硬停/急停手段，不能依赖关闭终端。

### 桌面高度标定

`calibrate_table_height.py` 是一键桌面高度标定脚本（抓取桌/放置桌通用）。
把箱子放在要标定的桌面上运行：

```bash
source /opt/ros/humble/setup.bash
source ~/demo3/install/setup.bash
ros2 run box_grasp_demo calibrate_table_height.py
```

脚本流程：

```text
输入卷尺实测桌面高度
→ 自动检查头部姿态（未低头会主动低头，保证相机能看到桌面）
→ 执行一次检测，反推视觉实测桌面 Z = 箱子中心 Z - 箱高/2
→ 读 TF 的 base_link 离地高度
→ 输出建议填写值: 桌面高度 = base_link离地 + 视觉桌面Z + 余量(≈0.005)
```

输出示例：

```text
卷尺实测桌面高度:           0.780 m
视觉实测桌面Z(base_link系): 0.060 m
base_link 离地高度:         0.815 m
推算桌面高度(建议填写值):   0.880 m
请将推算值填写到 config/box_grasp_ros2.yaml：
  抓取桌 → table_height:      0.880
  放置桌 → place_table_height: 0.880
```

使用建议：

- **抓取桌（`table_height`）**：直接填卷尺实测高度即可（抓取目标由视觉
  检测给出，卷尺误差不影响抓取）；如需核对系统值可用本脚本。
- **放置桌（`place_table_height`）**：必须用本脚本标定。放置时箱底高度
  由该值推算，系统坐标与卷尺值存在偏差，直接填卷尺值会压桌。换桌后
  重新跑一次脚本即可。

---

## 六、本地与机器人之间同步代码

本地代码路径：

```text
~/work/s2_demo3/src/box_grasp_demo/
```

机器人代码路径：

```text
s2_3_ros:/home/ubt/demo3/src/box_grasp_demo/
```

### 本地 -> 机器人

```bash
rsync -avc --exclude '__pycache__/' --exclude '*.pyc' \
  src/box_grasp_demo/ \
  s2_3_ros:/home/ubt/demo3/src/box_grasp_demo/
```

### 机器人 -> 本地

```bash
rsync -avc --exclude '__pycache__/' --exclude '*.pyc' \
  s2_3_ros:/home/ubt/demo3/src/box_grasp_demo/ \
  src/box_grasp_demo/
```

`-c` 按内容 checksum 比较，避免因文件修改时间不同而重复传输。

---

## 七、主要配置

感知与抓取参数在：

```text
src/box_grasp_demo/config/box_grasp_ros2.yaml
```

关键参数包括：

- `input_topic`：点云话题，默认 `/sensor/camera/stereo/pointcloud/raw`
- `target_frame`：输出位姿坐标系，默认 `base_link`
- `box_model` + `box_models`：箱子型号库。尺寸（`box_length / box_width /
  box_height`）和抓取几何（`tool_contact_below_top / side_clearance /
  pregrasp_distance / grasp_long_edge`）都在型号条目里，不在顶层。
  `base_box` 是完整默认值，其余型号只写要覆盖的字段。当前为 `small_box3`
  （400 × 300 × 230 mm）
- `table_height`：抓取桌面**离地绝对高度**，卷尺实测即可
- `support_height_min / support_height_max`：支撑平面的**相对偏移**范围（不是
  绝对高度）。桌面在 base_link 系的 Z = `table_height` − base_link 离地高度，
  过滤区间为该 Z 加上这两个偏移
- `workspace_min / workspace_max`：点云工作空间
- `pointcloud_offset`：点云整体平移（改 Z 时需同步调整 `support_height_*`）
- `place_table_height / base_link_height / place_x / place_y / place_yaw_deg`：放置位姿
- `pregrasp_q`：14 个手臂关节的默认预抓取姿态

IK 节点参数由 `launch/box_grasp_demo.launch.py` 传入，例如 `max_iters`、
`pos_threshold`、`ori_threshold`、`so_path` 等。

---

## 八、常见问题

### 本地 `./tools/run_offline_sim.sh` 报 colcon / ros2 找不到

先执行：

```bash
source /opt/ros/humble/setup.bash
```

### 下载 bag 后 `./tools/run_offline_sim.sh` 提取不到数据

确认文件存在：

```bash
ls -la robot_data/tf_analysis_current/
```

提取脚本固定读取：

```text
robot_data/tf_analysis_current/tf_analysis_current_0.db3
```

如果 bag 文件名不同，需要重命名，或修改
`tools/extract_current_bag.py` 中的 `bag` 路径。

### 图片下载没有返回

`save_img.sh` 会一直等下一帧 `/sensor/camera/stereo/color/raw`。确认相机话题正在
发布后，重新运行该脚本即可。
