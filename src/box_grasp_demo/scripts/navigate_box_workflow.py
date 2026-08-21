#!/usr/bin/python3.10
"""导航到箱子并复用 DetectBoxClient 完成抓取、搬运和放置。"""

import argparse
import signal
import sys
import threading
import traceback
from dataclasses import dataclass
from typing import Any, Callable, List, Optional, Sequence, Tuple


DEFAULT_ACTION_NAME = "/demo3/box_grasp_demo/detect_box"

EXIT_SUCCESS = 0
EXIT_FAILURE = 1
EXIT_DEPENDENCY = 3


class WorkflowInterrupted(Exception):
    """Raised after SIGINT/SIGTERM has requested a safe workflow stop."""

    def __init__(self, signum: int):
        self.signum = signum
        super().__init__(f"received signal {signum}")


class WorkflowStepFailed(Exception):
    """Raised when a workflow step fails or is declined."""


@dataclass
class StepRecord:
    name: str
    status: str = "PENDING"
    detail: str = ""


class RuntimeControl:
    """Tracks the active ubt Work so signals can stop it explicitly."""

    def __init__(self) -> None:
        self.stop_event = threading.Event()
        self.signal_number: Optional[int] = None
        self.current_work: Any = None
        self.current_wait_done: Optional[threading.Event] = None

    def set_current_work(self, work: Any, wait_done: threading.Event) -> None:
        self.current_work = work
        self.current_wait_done = wait_done

    def clear_current_work(self, work: Any) -> None:
        if self.current_work is work:
            if (self.current_wait_done is not None
                    and not self.current_wait_done.is_set()):
                # Keep the handle/event alive so final cleanup can wait before
                # destroying the SDK API object.
                return
            self.current_work = None
            self.current_wait_done = None

    def wait_until_idle(self, timeout: float = 10.0) -> bool:
        """等待 SDK waiter 收尾，防止 api.stop() 与后台线程并发。"""
        done = self.current_wait_done
        return done is None or done.wait(timeout=timeout)

    def request_stop(self, signum: int) -> bool:
        """Set the stop flag and directly stop the current ubt Work, if any."""
        repeated = self.signal_number is not None
        self.signal_number = signum
        self.stop_event.set()
        work = self.current_work
        if work is None:
            return False

        print(f"\n收到信号 {signum}，显式停止当前 ubt Work ...", flush=True)
        try:
            work.stop()
        except Exception as exc:  # best effort during signal handling
            print(f"停止当前 ubt Work 时发生异常: {exc}", file=sys.stderr, flush=True)
        if repeated:
            raise WorkflowInterrupted(signum)
        return True


class UbtWorkRunner:
    """Launches a skill and reliably waits for its WorkResult."""

    def __init__(self, ubt_robot: Any, skill: Any, control: RuntimeControl) -> None:
        self._ubt_robot = ubt_robot
        self._skill = skill
        self._control = control

    def run(self, skill_code: str, params: dict, name: str) -> bool:
        if self._control.stop_event.is_set():
            raise WorkflowInterrupted(self._control.signal_number or signal.SIGINT)

        print(f"  launch {skill_code}, params={params}")
        work = self._skill.launch(skill_code, params)
        if not work:
            print(f"  {name}: 技能启动失败")
            return False

        holder = {"result": None, "error": None}
        done = threading.Event()
        self._control.set_current_work(work, done)

        def wait_for_result() -> None:
            try:
                # Start the blocking wait before Work reaches a terminal state.
                # This preserves reliable result retrieval for fast skills such
                # as SetMap as well as for NavTo.
                holder["result"] = work.wait_until_finish()
            except Exception as exc:  # surfaced in the main thread below
                holder["error"] = exc
            finally:
                done.set()

        threading.Thread(target=wait_for_result, daemon=True).start()
        last_state = None

        try:
            while not done.is_set():
                if self._control.stop_event.is_set():
                    # The signal handler already calls stop() immediately; call
                    # it again here to cover stop requests set outside a signal.
                    try:
                        work.stop()
                    except Exception as exc:
                        print(f"  停止 Work 失败: {exc}", file=sys.stderr)
                    if not done.wait(timeout=10.0):
                        raise RuntimeError(
                            "停止技能后等待 WorkResult 超时（10s）")
                    raise WorkflowInterrupted(
                        self._control.signal_number or signal.SIGINT
                    )

                try:
                    state = work.state
                except Exception as exc:
                    print(f"  读取 Work 状态失败: {exc}")
                    return False

                if state != last_state:
                    print(f"  state: {self._state_name(state)}")
                    last_state = state
                done.wait(timeout=0.3)

            if self._control.stop_event.is_set():
                raise WorkflowInterrupted(
                    self._control.signal_number or signal.SIGINT
                )
            if holder["error"] is not None:
                print(f"  {name}: 等待执行结果失败: {holder['error']}")
                return False

            result = holder["result"]
            if result is None:
                try:
                    result = work.result
                except Exception:
                    result = None
            if result is None:
                print(f"  {name}: 执行结束但未取得 WorkResult")
                return False

            result_type = result.type
            if result_type == self._ubt_robot.WorkResultType.SUCCESS:
                print(f"  {name}: 成功, result={result.success_result}")
                return True
            if result_type == self._ubt_robot.WorkResultType.FAIL:
                print(f"  {name}: 失败, reason={result.fail_reason}")
                return False
            if result_type == self._ubt_robot.WorkResultType.STOPPED:
                print(f"  {name}: 已停止")
                return False

            print(f"  {name}: 未知结果类型 {result_type}")
            return False
        finally:
            self._control.clear_current_work(work)

    def _state_name(self, state: Any) -> str:
        names = {}
        work_state = self._ubt_robot.WorkState
        for attr in ("NONE", "UNDERWAY", "STOPPED", "PAUSED", "FINISHED"):
            value = getattr(work_state, attr, None)
            if value is not None:
                names[value] = attr
        return names.get(state, str(state))


class Workflow:
    """Executes state-changing steps with optional high-level confirmation."""

    def __init__(
        self,
        steps: Sequence[Tuple[str, Callable[[], bool]]],
        debug: bool,
        control: RuntimeControl,
    ) -> None:
        self._steps = list(steps)
        self._debug = debug
        self._control = control
        self.records = [StepRecord(name=name) for name, _ in self._steps]

    def run(self) -> None:
        for index, ((name, action), record) in enumerate(
            zip(self._steps, self.records), start=1
        ):
            if self._control.stop_event.is_set():
                record.status = "INTERRUPTED"
                raise WorkflowInterrupted(
                    self._control.signal_number or signal.SIGINT
                )

            print(f"\n--- [{index}/{len(self._steps)}] {name} ---")
            if self._debug and not self._confirm(name):
                record.status = "CANCELLED"
                record.detail = "debug 模式下用户未确认"
                raise WorkflowStepFailed(f"用户取消步骤: {name}")

            record.status = "RUNNING"
            try:
                succeeded = action()
            except WorkflowInterrupted:
                record.status = "INTERRUPTED"
                record.detail = "收到终止信号"
                raise
            except Exception as exc:
                record.status = "FAILED"
                record.detail = str(exc)
                raise WorkflowStepFailed(f"{name} 异常: {exc}") from exc

            if not succeeded:
                record.status = "FAILED"
                record.detail = "步骤返回失败"
                raise WorkflowStepFailed(f"步骤失败: {name}")
            record.status = "SUCCESS"

    def _confirm(self, name: str) -> bool:
        try:
            answer = input(
                f"[DEBUG] 高层步骤“{name}”将改变机器人状态，是否执行？[y/N] "
            )
        except EOFError:
            print("未收到确认输入，取消该步骤。")
            return False
        return answer.strip().lower() in ("y", "yes")


def positive_float(value: str) -> float:
    number = float(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("必须大于 0")
    return number


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="独立导航 + DetectBoxClient 抓放编排程序"
    )
    parser.add_argument("--map-id", default="lqx3", help="地图 ID（默认: lqx3）")
    parser.add_argument(
        "--pick-target", default="box1", help="抓取导航点 targetId（默认: box1）"
    )
    parser.add_argument(
        "--mid-target", default="mid1", help="搬运中间点 targetId（默认: mid1）"
    )
    parser.add_argument(
        "--place-target", default="put1", help="放置导航点 targetId（默认: put1）"
    )
    parser.add_argument(
        "--address",
        default="0.0.0.0:51000",
        help="ubt_robot SDK 地址（默认: 0.0.0.0:51000）",
    )
    parser.add_argument(
        "--action-name",
        default=DEFAULT_ACTION_NAME,
        help=f"DetectBox action 名称（默认: {DEFAULT_ACTION_NAME}）",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="每个高层状态变更前确认；机械臂内部路点也逐点确认",
    )
    parser.add_argument(
        "--connect-timeout",
        type=positive_float,
        default=10.0,
        metavar="SECONDS",
        help="连接机器人超时秒数（默认: 10）",
    )
    return parser.parse_args(argv)


def normalize_address(address: str) -> str:
    return address if ":" in address else f"{address}:51000"


def load_ubt_sdk() -> Tuple[Any, Any, Any, Any]:
    """Load ubt_robot lazily so argparse --help works without the SDK."""
    try:
        import ubt_robot
        from ubt_robot import Api, Config, Skill
    except (ImportError, OSError) as exc:
        print(f"错误: ubt_robot SDK 缺失或无法加载: {exc}", file=sys.stderr)
        print(
            "项目已附带 x86_64/aarch64 的 ubt_robot wheel，正常情况下会由 "
            "colcon build 安装到工作区 install 前缀。",
            file=sys.stderr,
        )
        print("请确认 /usr/bin/python3.10 存在（wheel 仅支持 CPython 3.10），然后执行：",
              file=sys.stderr)
        print("  sudo apt install libmosquitto1", file=sys.stderr)
        print(
            "  colcon build --packages-up-to box_grasp_demo --symlink-install",
            file=sys.stderr,
        )
        print("  source install/setup.bash", file=sys.stderr)
        raise
    return ubt_robot, Api, Config, Skill


def get_detected_ik_data(client: Any) -> Any:
    """Strictly return IK data from the current successful action result."""
    result = getattr(client, "_result", None)
    if result is None:
        raise WorkflowStepFailed("DetectBox 未返回 result")
    if getattr(result, "success", None) is not True:
        message = getattr(result, "message", "")
        suffix = f": {message}" if message else ""
        raise WorkflowStepFailed(f"DetectBox result.success 不是 True{suffix}")
    ik_data = getattr(result, "ik_data", None)
    if not ik_data:
        raise WorkflowStepFailed("DetectBox 成功，但 result.ik_data 为空")
    return ik_data


def detect_box(client: Any, detected: dict) -> bool:
    """Run detection and cache this result's strictly validated IK data."""
    if not client.send_goal():
        return False
    client.print_result()
    detected["ik_data"] = get_detected_ik_data(client)
    return True


def build_steps(
    args: argparse.Namespace,
    runner: UbtWorkRunner,
    client: Any,
) -> List[Tuple[str, Callable[[], bool]]]:
    """Build the fixed fail-fast navigation, pick, and place workflow.

    流程开头先激活一次 biped home，之后下蹲直接发绝对下蹲量。
    非调试模式：手臂段合并为单个 /v1/motions 请求（抓取段 = 预抓取/抓取/
    抬起/收回 + 站直；放置段 = 放置/返回拍照 + 站直），下蹲为独立前置
    步骤（放置 IK 需在下蹲后的 base_link 高度下重算）；调试模式：保持
    分步确认。
    """
    detected = {"ik_data": None}

    if args.debug:
        pick_steps: List[Tuple[str, Callable[[], bool]]] = [
            ("抓取前下蹲", lambda: client.prepare_crouch("grasp")),
            ("执行抓取段（预抓取/抓取/抬起/收回）",
             lambda: client.run_pick_sequence(detected["ik_data"])),
            ("恢复导航高度（抓取后）", client.prepare_navigation_height),
        ]
        place_steps: List[Tuple[str, Callable[[], bool]]] = [
            ("放置前下蹲", lambda: client.prepare_crouch("place")),
            ("重新计算并执行当前放置段", client.run_current_place_sequence),
            ("恢复导航高度（放置后）", client.prepare_navigation_height),
        ]
    else:
        # 拍照前已下蹲，抓取段不再下蹲；合并请求末尾用 motions 下蹲量 0
        # 站直收尾，因此无需单独的"恢复导航高度"步骤。
        pick_steps = [
            ("合并：抓取段（预抓取/抓取/抬起/收回）+站直",
             lambda: client.run_pick_combined(detected["ik_data"])),
        ]
        # 放置 IK 需要在下蹲后的 base_link 高度下重新解算，因此下蹲必须
        # 作为独立前置步骤，再执行合并的"放置段+站直"。
        place_steps = [
            ("放置前下蹲", lambda: client.prepare_crouch("place")),
            ("合并：放置段+站直", client.run_place_combined),
        ]

    return [
        # 任务一开始先激活 biped home（整个流程只执行一次），后续下蹲
        # 直接发绝对下蹲量；不再使用 SetLegMode STAND（home 已激活站立）。
        ("激活 biped home（流程开始，仅一次）", client.activate_biped_home),
        (
            f"SetMap {args.map_id} (A000012)",
            lambda: runner.run(
                "A000012", {"mapId": args.map_id}, f"SetMap {args.map_id}"
            ),
        ),
        (
            "Relocation global (A000013)",
            lambda: runner.run(
                "A000013", {"mode": "global"}, "Relocation global"
            ),
        ),
        (
            f"NavTo {args.pick_target} (A000002)",
            lambda: runner.run(
                "A000002",
                {"targetId": args.pick_target},
                f"NavTo {args.pick_target}",
            ),
        ),
        # 抓取前腰部后仰（waist 组件，默认 0.32rad≈18.3°），抓取/检测及抓取后导航
        # 全程保持，放置完成后恢复。
        ("腰部后仰（抓取前）", client.tilt_waist_back),
        ("准备拍照姿态", client.prepare_photo_position),
        ("拍照前下蹲", lambda: client.prepare_crouch("photo")),
        ("检测箱子", lambda: detect_box(client, detected)),
        *pick_steps,
        (
            f"NavTo {args.mid_target} (A000002)",
            lambda: runner.run(
                "A000002",
                {"targetId": args.mid_target},
                f"NavTo {args.mid_target}",
            ),
        ),
        (
            f"NavTo {args.place_target} (A000002)",
            lambda: runner.run(
                "A000002",
                {"targetId": args.place_target},
                f"NavTo {args.place_target}",
            ),
        ),
        *place_steps,
        ("腰部恢复（放置后）", client.tilt_waist_center),
        ("收起双臂", client.retract_arms),
    ]


def print_configuration(args: argparse.Namespace, address: str) -> None:
    print("=== Navigate Box Workflow ===")
    print(f"  地图:       {args.map_id}")
    print(f"  抓取点:     {args.pick_target}")
    print(f"  中间点:     {args.mid_target}")
    print(f"  放置点:     {args.place_target}")
    print(f"  SDK 地址:   {address}")
    print(f"  Action:     {args.action_name}")
    print(f"  连接超时:   {args.connect_timeout:g} s")
    print(f"  Debug:      {'开启' if args.debug else '关闭'}")


def print_summary(
    records: Sequence[StepRecord],
    outcome: str,
    failure_reason: str,
    exit_code: int,
) -> None:
    labels = {
        "SUCCESS": "✓",
        "FAILED": "✗",
        "CANCELLED": "×",
        "INTERRUPTED": "!",
        "RUNNING": "!",
        "PENDING": "—",
    }
    print("\n=== Workflow Summary ===")
    if records:
        for record in records:
            suffix = f" — {record.detail}" if record.detail else ""
            print(f"  [{labels.get(record.status, '?')}] {record.name}{suffix}")
    else:
        print("  [—] 工作流未开始")
    print(f"Status: {outcome}")
    if failure_reason:
        print(f"Reason: {failure_reason}")
    print(f"Exit code: {exit_code}")


def main(argv: Optional[Sequence[str]] = None) -> int:
    # Parse before importing ubt_robot/ROS modules.  In particular, --help must
    # remain usable on hosts where the architecture-specific SDK is absent.
    args = parse_args(argv)
    address = normalize_address(args.address)
    print_configuration(args, address)

    control = RuntimeControl()
    workflow: Optional[Workflow] = None
    api = None
    client = None
    rclpy = None
    ros_initialized = False
    exit_code = EXIT_FAILURE
    outcome = "FAILED"
    failure_reason = ""

    old_sigint = signal.getsignal(signal.SIGINT)
    old_sigterm = signal.getsignal(signal.SIGTERM)

    def signal_handler(signum: int, _frame: Any) -> None:
        had_work = control.request_stop(signum)
        if not had_work:
            print(f"\n收到信号 {signum}，正在终止流程 ...", flush=True)
            raise WorkflowInterrupted(signum)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        try:
            ubt_robot, Api, Config, Skill = load_ubt_sdk()
        except (ImportError, OSError) as exc:
            exit_code = EXIT_DEPENDENCY
            outcome = "DEPENDENCY ERROR"
            failure_reason = f"ubt_robot SDK 无法加载: {exc}"
            raise WorkflowStepFailed(failure_reason) from exc

        try:
            import rclpy as rclpy_module
            from trigger_detect_box import DetectBoxClient
        except (ImportError, OSError) as exc:
            exit_code = EXIT_DEPENDENCY
            outcome = "DEPENDENCY ERROR"
            failure_reason = (
                f"ROS 2/box_grasp_demo 依赖无法加载: {exc}；"
                "请先 source ROS 2 与工作空间 setup.bash"
            )
            raise WorkflowStepFailed(failure_reason) from exc

        rclpy = rclpy_module
        rclpy.init(args=[])
        ros_initialized = True
        client = DetectBoxClient(args.action_name, debug=args.debug)

        api = Api()
        config = Config()
        config.address = address
        config.api_id = "demo"
        config.token = ""
        api.initialize(config)
        api.start()
        print(f"\n连接 ubt_robot SDK: {address}")
        timeout_ms = max(1, int(args.connect_timeout * 1000))
        if not api.wait_until_connected(timeout_ms=timeout_ms):
            failure_reason = f"连接机器人超时（{args.connect_timeout:g} s）"
            raise WorkflowStepFailed(failure_reason)

        runner = UbtWorkRunner(ubt_robot, Skill(api), control)
        workflow = Workflow(
            build_steps(args, runner, client),
            debug=args.debug,
            control=control,
        )
        workflow.run()
        outcome = "SUCCESS — all steps completed"
        exit_code = EXIT_SUCCESS
    except WorkflowInterrupted as exc:
        exit_code = 128 + exc.signum
        outcome = "INTERRUPTED"
        failure_reason = f"收到信号 {exc.signum}"
    except WorkflowStepFailed as exc:
        if not failure_reason:
            failure_reason = str(exc)
        if outcome != "DEPENDENCY ERROR":
            outcome = "ABORTED"
            exit_code = EXIT_FAILURE
    except KeyboardInterrupt:
        # Fallback for an interrupt delivered before/while handlers are changed.
        control.request_stop(signal.SIGINT)
        exit_code = 128 + signal.SIGINT
        outcome = "INTERRUPTED"
        failure_reason = "收到 SIGINT"
    except Exception as exc:
        failure_reason = f"未处理异常: {exc}"
        outcome = "ERROR"
        exit_code = EXIT_FAILURE
        print(f"错误: {failure_reason}", file=sys.stderr)
        if args.debug:
            traceback.print_exc()
    finally:
        cleanup_errors = []
        sdk_idle = control.wait_until_idle(timeout=10.0)
        if not sdk_idle:
            cleanup_errors.append(
                "UBT Work 等待线程未结束；为避免 SDK 销毁竞态，跳过 api.stop()")
        if api is not None:
            if sdk_idle:
                try:
                    api.stop()
                    print("\nubt_robot Api 已停止。")
                except Exception as exc:
                    cleanup_errors.append(f"停止 Api 失败: {exc}")

        if client is not None:
            try:
                client.cancel_active_goal()
            except Exception as exc:
                cleanup_errors.append(f"取消 DetectBox goal 失败: {exc}")
            try:
                client.destroy_node()
                print("ROS node 已销毁。")
            except Exception as exc:
                cleanup_errors.append(f"销毁 ROS node 失败: {exc}")

        if rclpy is not None and ros_initialized:
            try:
                if rclpy.ok():
                    rclpy.shutdown()
                print("rclpy 已 shutdown。")
            except Exception as exc:
                cleanup_errors.append(f"rclpy shutdown 失败: {exc}")

        if cleanup_errors:
            print("清理阶段发生错误：", file=sys.stderr)
            for error in cleanup_errors:
                print(f"  - {error}", file=sys.stderr)
            if exit_code == EXIT_SUCCESS:
                exit_code = EXIT_FAILURE
                outcome = "CLEANUP ERROR"
                failure_reason = "; ".join(cleanup_errors)

        signal.signal(signal.SIGINT, old_sigint)
        signal.signal(signal.SIGTERM, old_sigterm)
        print_summary(
            workflow.records if workflow is not None else [],
            outcome,
            failure_reason,
            exit_code,
        )

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
