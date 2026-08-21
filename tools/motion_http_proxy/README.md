# Walker S2 HTTP 运动代理

本目录是当前项目自带的 HTTP→ROSA 运动代理。部署后，抓放程序通过
`http://192.168.11.2:12300` 调用 Walker `motion` 容器里的
`manipulation_task_manager`，无需依赖工作区外的代理仓库。

> 代理不做轨迹碰撞检查、关节限位复核、力控或急停。HTTP 200 只表示 ROSA CLI
> 正常返回，不表示机器人已经到位。首次运行必须清空运动区并使用 debug 流程。

## 文件

| 文件 | 用途 |
| --- | --- |
| `motion_http_proxy` | 预编译好的二进制，部署时直接同步这个文件即可 |
| `run_motion_http_proxy.sh` | 加载 Walker 环境并前台启动 |
| `call_action.sh` | 手工调用 `move_biped_home`；必须显式 `--execute` |
| `call_motion.sh` | 手工发送关节目标；必须显式 `--execute` |
| `motion.example.json` | 仅展示字段的占位模板，不能直接作为真机目标 |
| `motion/` | 5 个可直接使用的示例动作 JSON，见下面「示例动作」一节 |

## 接口

- `GET /healthz`：进程健康检查，不调用 ROSA。
- `POST /v1/call_action`：body 为 `{"action":"..."}`。
- `POST /v1/motions`：body 直接是包含 `component_names` 和 `goals` 的运动 JSON。

同一时间只执行一个 POST 请求，并发请求返回 HTTP 409。服务端设置
`MOTION_PROXY_API_KEY` 后，三个接口都必须携带相同的 `X-API-Key`。

## 1. 安装到 Walker 设备

目标部署约定：宿主机 `/debug/proxy_server` 挂载到 `motion` 容器的
`/proxy_server`。目标机地址 `192.168.11.2`，用户名 `walker`。二进制已经预编译好，
不需要在设备上重新编译。从本项目根目录同步文件：

```bash
cd ~/work/s2_demo3
rsync -av --delete tools/motion_http_proxy/ \
  walker@192.168.11.2:/debug/proxy_server/
```

`--delete` 只作用于目标 `/debug/proxy_server/`。不希望删除目标旧文件时去掉该参数。

登录设备确认二进制可执行：

```bash
ssh walker@192.168.11.2
cd /debug/proxy_server
chmod +x run_motion_http_proxy.sh call_action.sh call_motion.sh motion_http_proxy
test -x ./motion_http_proxy
```

## 2. 前台启动

在 `motion` 容器内：

```bash
cd /proxy_server
export HTTP_PROXY_HOST=0.0.0.0
export HTTP_PROXY_PORT=12300
export MOTION_PROXY_API_KEY='替换为现场密钥'
export MOTION_PROXY_LOG_COMMAND=0
exec ./run_motion_http_proxy.sh
```

可配置变量：

| 变量 | 默认值 | 说明 |
| --- | --- | --- |
| `WALKER_SETUP` | `/opt/walker/setup.bash` | Walker 环境脚本 |
| `PROXY_BIN` | 同目录 `motion_http_proxy` | 代理二进制 |
| `HTTP_PROXY_HOST` | 启动脚本为 `0.0.0.0` | 监听 IPv4 地址 |
| `HTTP_PROXY_PORT` | `12300` | 监听端口 |
| `ROSA_BIN` | `rosa` | ROSA CLI 路径 |
| `MOTION_PROXY_API_KEY` | 空 | 非空时启用全接口鉴权 |
| `MOTION_PROXY_LOG_COMMAND` | 空 | `1/true` 时记录完整调用参数 |

日志参数会暴露关节目标，仅临时调试使用。前台停止可按 `Ctrl+C` 结束代理进程；这不会
撤回已经交给机器人执行的动作。

## 3. UDoKe / Docker 模块启动

在 UDoKe 的 `motion` 分组中增加模块，使用宿主机目录挂载：

```yaml
proxy_server:
  extends: mc_common
  command: "/proxy_server/run_motion_http_proxy.sh"
  ports:
    - "12300:12300"
  volumes:
    - /debug/proxy_server:/proxy_server
  environment:
    HTTP_PROXY_HOST: "0.0.0.0"
    HTTP_PROXY_PORT: "12300"
    MOTION_PROXY_API_KEY: "替换为现场密钥"
    MOTION_PROXY_LOG_COMMAND: "0"
```

重启/停止使用 UDoKe 的模块控制。若直接用 Docker 管理，可先查容器名，再查看日志：

```bash
docker ps --format '{{.Names}}' | grep proxy_server
docker logs -f walker-motion.proxy_server-1
```

本目录不提供 systemd unit、Dockerfile 或 Compose 文件；不要声称已配置开机自启。

## 4. 只做健康检查

客户端与服务端使用相同密钥：

```bash
export BASE_URL=http://192.168.11.2:12300
export MOTION_PROXY_API_KEY='替换为现场密钥'

curl_args=(-fsS)
if [[ -n "${MOTION_PROXY_API_KEY:-}" ]]; then
  curl_args+=(-H "X-API-Key: ${MOTION_PROXY_API_KEY}")
fi
curl "${curl_args[@]}" "${BASE_URL}/healthz"
```

成功返回 `{"status":"ok"}`。这只证明代理可达。

## 5. 动作接口（真机会运动）

两个 helper 默认不发送请求，缺少 `--execute` 时退出。只有在现场人员确认安全后才能用：

```bash
cd tools/motion_http_proxy

# 会让机器人执行 move_biped_home
BASE_URL=http://192.168.11.2:12300 ./call_action.sh --execute

# 必须提供已经逐项审核的真实 JSON；不要使用占位模板
BASE_URL=http://192.168.11.2:12300 \
  ./call_motion.sh --execute /path/to/reviewed-motion.json
```

本项目自动流程读取 `MOTION_BASE_URL`（不是 `BASE_URL`）和
`MOTION_PROXY_API_KEY`：

```bash
export MOTION_BASE_URL=http://192.168.11.2:12300
export MOTION_PROXY_API_KEY='替换为现场密钥'
```

### 示例动作

`motion.example.json` 是纯占位模板（`call_motion.sh` 会拒绝执行它）。`motion/`
下则是 5 个字段完整、可直接发送的动作 JSON：

| 文件 | `component_names`（维度） | 内容 |
| --- | --- | --- |
| `motion_head.json` | `head`（2） | 低头到拍照角度 `[yaw=0, pitch=-0.65]` |
| `waist.json` | `waist`（2） | 腰部后仰 `[yaw=0, pitch=0.32]`（≈18.3°），单路点 |
| `down_10cm.json` | `biped`（6） | 下蹲 10 cm，单路点 |
| `motion_ready.json` | `left_arm,right_arm,head`（7+7+2） | 手臂零位 → 拍照位，3 个路点 |
| `motion_ready_to_home.json` | `waist,left_arm,right_arm,head`（2+7+7+2） | 上一条的逆序，收回到全零位并让 waist 归零 |

关节顺序：手臂为 `[shoulder_pitch, shoulder_roll, shoulder_yaw, elbow_roll,
elbow_yaw, wrist_pitch, wrist_roll]`，左臂在前、右臂在后；`head` 为
`[head_yaw, head_pitch]`；`waist` 为 `[waist_yaw, waist_pitch]`；`biped` 为 6 维，
**第三个元素是下蹲量**。

几个和主程序对得上的点：

- `down_10cm.json` 的 `-0.1` 是**相对站立位的绝对量**，负值向下，站直填 `0`；
  这与抓放流程里 biped 下蹲的约定一致，不是增量。
- `motion_head.json` 的 `-0.65` 就是 `calibrate_table_height.py` 里的
  `HEAD_PITCH_TARGET`，也是拍照位的 head 角度。
- `waist.json` 的 `0.32` 就是 `WAIST_TILT_BACK_RAD`，等价于
  `trigger_detect_box.py` 的 `tilt_waist_back()`。抓放流程在抓取前后仰让重心后移，
  抓取、检测和搬运全程保持，放置完成后再由 `tilt_waist_center()` 回正。
  **要回正就把 `pitch` 改成 `0` 再发**——目录里没有单独的回正 JSON。
- `motion_ready.json` 的终点路点与 `config/box_grasp_ros2.yaml` 的 `pregrasp_q`
  完全一致，对应 `trigger_detect_box.py` 的 `MOTION_READY_BODY`（该常量的代码
  注释也注明「内容对应 motion_ready.json」）。两者路点相同，但常量用
  `vel_scale=0.2`，JSON 里是 `0.1`。
- `motion_ready_to_home.json` 对应 `MOTION_RETRACT_BODY`（流程末尾「收起双臂」），
  路点和 `vel_scale` 完全一致，JSON 版额外在每个路点前加了 waist 的两个 `0`。

**这些 JSON 只供人工单步调试。** 自动流程不读取这些文件，用的是上面那几个代码内
常量；改 JSON 不会影响 `trigger_detect_box.py` / `navigate_box_workflow.py` 的行为。

用法（会真实运动，先清空运动区并确认急停手段）：

```bash
cd tools/motion_http_proxy
export BASE_URL=http://192.168.11.2:12300
export MOTION_PROXY_API_KEY='替换为现场密钥'

./call_motion.sh --execute motion/motion_head.json          # 先用 head 验证链路
./call_motion.sh --execute motion/waist.json               # 腰部后仰 18.3°
./call_motion.sh --execute motion/motion_ready.json         # 双臂到拍照位
./call_motion.sh --execute motion/motion_ready_to_home.json # 收回零位
```

`motion_ready.json`、`motion_ready_to_home.json` 和 `down_10cm.json` 会带动大关节，
建议先跑 `motion_head.json` 确认代理与鉴权通了再继续。

## 6. HTTP 状态与排障

| 状态/现象 | 含义与处理 |
| --- | --- |
| 400 | JSON/Content-Length 不合法，或 motion 缺少必要字段 |
| 401 | API key 未带或不一致 |
| 404 | 路径或 HTTP 方法错误 |
| 409 | 已有动作请求在执行；不要并发或盲目重发 |
| 415 | POST 未使用 `application/json` |
| 502 | ROSA CLI 失败；检查响应 `output` 和容器日志 |
| `missing Walker environment` | `WALKER_SETUP` 不存在或不在正确容器 |
| `rosa was not found` | Walker 环境未正确加载 |
| `Address already in use` | 端口已有代理实例，先确认后再重启 |
| HTTP 200 但未到位 | 通过 `/mc/joint_states`、TF 和机器人告警独立确认 |

代理没有动作撤回接口。客户端超时或 `Ctrl+C` 后机器人仍可能继续执行已接收动作；必须
使用现场验证过的硬停/急停方式，不能靠关闭终端代替。
