"""
Walker S2 双臂逆运动学求解器

基于 Pinocchio + CasADi + IPOPT 的数值优化 IK。
锁定腿部、腰部和夹爪关节，仅对双臂 14 个关节（每臂 7 DOF）进行求解。

特性：
- 双腕位姿（位置 + 方向）同时求解
- 支持 .so 代码生成以加速实时执行
- 迭代热启动逐步收敛
- 可配置的代价权重
"""

import os
import sys
import time

# 确保 conda 环境的 pinocchio（含 casadi 绑定）优先于 apt 版本。
# ROS2 setup.bash 会将 /opt/ros/humble 加入 PYTHONPATH，其中 apt 版
# pinocchio（无 cpin）会覆盖 conda 版本，导致 pinocchio.casadi 导入失败。
# 此处从解释器路径反推 conda site-packages，插入 sys.path 最前面。
_py_ver = f"python{sys.version_info.major}.{sys.version_info.minor}"
for _conda_prefix in [
    os.environ.get("CONDA_PREFIX", ""),
    os.path.dirname(os.path.dirname(sys.executable)),   # .../envs/ik/bin/python → .../envs/ik
]:
    _conda_site = os.path.join(_conda_prefix, "lib", _py_ver, "site-packages")
    if os.path.isdir(_conda_site):
        sys.path.insert(0, _conda_site)
        break

import numpy as np

import casadi
import pinocchio as pin
from pinocchio import casadi as cpin


# ---------------------------------------------------------------------------
# 默认锁定关节列表（Walker S2 除双臂外的所有关节）
# ---------------------------------------------------------------------------
DEFAULT_JOINTS_TO_LOCK = [
    # 腿部 (12)
    "L_hip_roll_joint", "L_hip_yaw_joint", "L_hip_pitch_joint",
    "L_knee_pitch_joint", "L_ankle_pitch_joint", "L_ankle_roll_joint",
    "R_hip_roll_joint", "R_hip_yaw_joint", "R_hip_pitch_joint",
    "R_knee_pitch_joint", "R_ankle_pitch_joint", "R_ankle_roll_joint",
    # 腰部 (2)
    "waist_yaw_joint", "waist_pitch_joint",
    # 头部 (2): S2 v1 has movable yaw/pitch.  The dual-arm IK solves only
    # the 14 arm joints, so these must be locked at the reference pose.
    "head_yaw_joint", "head_pitch_joint",
    # v1 end-effector branch (4): the gripper sliders are visual/tool joints,
    # not IK degrees of freedom.  Lock them at zero so the reduced model is
    # exactly the two 7-DOF arms.
    "left_R_gripper_joint", "left_L_gripper_joint",
    "right_R_gripper_joint", "right_L_gripper_joint",
]

# 手臂关节名称（按运动链顺序）
LEFT_ARM_JOINTS = [
    "L_shoulder_pitch_joint", "L_shoulder_roll_joint", "L_shoulder_yaw_joint",
    "L_elbow_roll_joint", "L_elbow_yaw_joint",
    "L_wrist_pitch_joint", "L_wrist_roll_joint",
]

RIGHT_ARM_JOINTS = [
    "R_shoulder_pitch_joint", "R_shoulder_roll_joint", "R_shoulder_yaw_joint",
    "R_elbow_roll_joint", "R_elbow_yaw_joint",
    "R_wrist_pitch_joint", "R_wrist_roll_joint",
]


class WalkerArmIK:
    """Walker S2 双臂逆运动学求解器。

    Parameters
    ----------
    urdf_path : str
        Walker S2 URDF 模型文件路径。
    mesh_path : str, optional
        URDF 中 mesh 文件的搜索路径（目录）。
    package_dirs : list[str], optional
        用于解析 package:// 前缀的 ROS 包搜索路径列表。
        例如：["/path/to/ros_ws/src", "/path/to/ros_ws/install/xxx/share"]
    joints_to_lock : list[str], optional
        需要锁定的关节名称列表。默认锁定腿部、腰部和夹爪。
    jit : bool
        是否启用 CasADi JIT 编译。
    so_path : str, optional
        预编译 .so 文件路径。若提供且存在则跳过 Python 求解器构建。
    """

    # 代价权重
    POS_WEIGHT = 60.0       # 位置误差权重
    ROT_WEIGHT = 3.0        # 旋转误差权重
    REG_WEIGHT = 1e-4       # 正则化（最小化关节角度）
    SMOOTH_WEIGHT = 0.01    # 平滑性（接近初始值）

    def __init__(
        self,
        urdf_path: str,
        mesh_path: str | None = None,
        package_dirs: list[str] | None = None,
        joints_to_lock: list[str] | None = None,
        jit: bool = False,
        so_path: str | None = None,
    ):
        self.urdf_path = urdf_path
        self.mesh_path = mesh_path or os.path.dirname(urdf_path)
        self.use_jit = jit

        if joints_to_lock is None:
            joints_to_lock = DEFAULT_JOINTS_TO_LOCK

        # ---- 1. 加载并简化模型 ----
        # Pinocchio 的 BuildFromURDF 第二个位置参数即为 package_dirs，
        # 用于解析 URDF 中 package:// 开头的 mesh 路径。
        # 将 mesh_path 和显式的 package_dirs 合并。
        if package_dirs is None:
            package_dirs = []
        elif isinstance(package_dirs, str):
            package_dirs = [package_dirs]
        all_package_dirs = [self.mesh_path] + list(package_dirs)
        self.robot = pin.RobotWrapper.BuildFromURDF(
            urdf_path, all_package_dirs,
        )

        # Fixed joints in a URDF become frames, not Pinocchio model joints.
        # Keep a shared lock list compatible with both the old model and the
        # S2 v1 model by filtering names absent from the loaded model.
        available_joints = set(self.robot.model.names)
        self.joints_to_lock = [name for name in joints_to_lock
                               if name in available_joints]
        ignored_locks = [name for name in joints_to_lock
                         if name not in available_joints]
        if ignored_locks:
            print("[WalkerArmIK] 忽略 URDF 中不存在/固定的锁定关节: "
                  + ", ".join(ignored_locks))

        self.reduced_robot = self.robot.buildReducedRobot(
            list_of_joints_to_lock=self.joints_to_lock,
            reference_configuration=np.zeros(self.robot.model.nq),
        )

        # ---- 2. 添加末端执行器 Frame ----
        self._L_wrist_joint_id = self.reduced_robot.model.getJointId("L_wrist_roll_joint")
        self._R_wrist_joint_id = self.reduced_robot.model.getJointId("R_wrist_roll_joint")

        self.reduced_robot.model.addFrame(
            pin.Frame("L_ee", self._L_wrist_joint_id,
                      pin.SE3(np.eye(3), np.zeros(3)),
                      pin.FrameType.OP_FRAME))
        self.reduced_robot.model.addFrame(
            pin.Frame("R_ee", self._R_wrist_joint_id,
                      pin.SE3(np.eye(3), np.zeros(3)),
                      pin.FrameType.OP_FRAME))

        self.reduced_robot.data = self.reduced_robot.model.createData()

        self.L_hand_id = self.reduced_robot.model.getFrameId("L_ee")
        self.R_hand_id = self.reduced_robot.model.getFrameId("R_ee")

        # ---- 3. 关节索引映射 ----
        self.joint_indices = self._build_joint_indices()

        # ---- 4. 初始值 ----
        self.init_data = np.zeros(self.reduced_robot.model.nq)

        # ---- 5. 构建求解器 ----
        if so_path and os.path.exists(so_path):
            print(f"[WalkerArmIK] 从 {so_path} 加载外部求解器...")
            self.ik_solver_func = casadi.external('ik_solver', so_path)
            print("[WalkerArmIK] 外部求解器加载成功。")
        else:
            self._build_solver()

    # ------------------------------------------------------------------
    # 内部工具
    # ------------------------------------------------------------------

    def _build_joint_indices(self) -> dict[str, int]:
        """构建关节名称 → q 向量索引的映射。"""
        indices = {}
        for name in self.reduced_robot.model.names:
            if name == "universe":
                continue
            j_id = self.reduced_robot.model.getJointId(name)
            idx = self.reduced_robot.model.joints[j_id].idx_q
            indices[name] = idx
        return indices

    def _build_solver(self) -> None:
        """构建 CasADi + IPOPT NLP 求解器。"""
        nq = self.reduced_robot.model.nq

        cmodel = cpin.Model(self.reduced_robot.model)
        cdata = cmodel.createData()

        # 符号变量
        q = casadi.SX.sym("q", nq)
        q_init = casadi.SX.sym("q_init", nq)
        target_l = casadi.SX.sym("target_l", 4, 4)
        target_r = casadi.SX.sym("target_r", 4, 4)

        # 正向运动学
        cpin.framesForwardKinematics(cmodel, cdata, q)

        # 位置误差
        e_p_l = cdata.oMf[self.L_hand_id].translation - target_l[:3, 3]
        e_p_r = cdata.oMf[self.R_hand_id].translation - target_r[:3, 3]

        # 旋转误差 (SO(3) 对数映射)
        e_r_l = cpin.log3(cdata.oMf[self.L_hand_id].rotation @ target_l[:3, :3].T)
        e_r_r = cpin.log3(cdata.oMf[self.R_hand_id].rotation @ target_r[:3, :3].T)

        # 代价函数
        cost = (
            self.POS_WEIGHT * (casadi.sumsqr(e_p_l) + casadi.sumsqr(e_p_r))
            + self.ROT_WEIGHT * (casadi.sumsqr(e_r_l) + casadi.sumsqr(e_r_r))
            + self.REG_WEIGHT * casadi.sumsqr(q)
            + self.SMOOTH_WEIGHT * casadi.sumsqr(q - q_init)
        )

        # 关节限位（处理无界情况）
        low = self.reduced_robot.model.lowerPositionLimit.copy()
        high = self.reduced_robot.model.upperPositionLimit.copy()
        low[low < -1e10] = -100.0
        high[high > 1e10] = 100.0

        # IPOPT 选项
        jit_opts = {"flags": ["-O3"], "verbose": False}
        opts = {
            "ipopt": {
                "print_level": 0,
                "max_iter": 12,
                "tol": 1e-3,
                "acceptable_tol": 1e-3,
                "linear_solver": "mumps",
            },
            "print_time": False,
            "expand": True,
            "calc_lam_p": False,        # 不计算拉格朗日乘子，避免 NaN 警告
            "jit": self.use_jit,
            "compiler": "shell",
            "jit_options": jit_opts,
        }

        nlp = {"x": q, "f": cost, "p": casadi.vertcat(
            casadi.vec(target_l), casadi.vec(target_r), q_init)}

        solver = casadi.nlpsol("solver", "ipopt", nlp, opts)

        # 封装为可导出的 Function
        p_vec = casadi.vertcat(
            target_l.reshape((-1, 1)),
            target_r.reshape((-1, 1)),
            q_init,
        )
        res = solver(x0=q_init, p=p_vec, lbx=low, ubx=high)
        self.ik_solver_func = casadi.Function(
            "ik_solver", [target_l, target_r, q_init], [res["x"]]
        )

    # ------------------------------------------------------------------
    # 公共 API
    # ------------------------------------------------------------------

    def get_arm_q(self, q: np.ndarray, side: str = "R") -> np.ndarray:
        """从完整配置向量中提取指定手臂的 7 个关节角度。

        Parameters
        ----------
        q : np.ndarray
            简化模型的完整配置向量。
        side : str
            "L" 或 "R"，指定左手或右手。

        Returns
        -------
        np.ndarray
            7 个关节角度 [shoulder_pitch, shoulder_roll, shoulder_yaw,
                           elbow_roll, elbow_yaw, wrist_pitch, wrist_roll]
        """
        joints = LEFT_ARM_JOINTS if side.upper() == "L" else RIGHT_ARM_JOINTS
        return np.array([q[self.joint_indices[name]] for name in joints])

    def solve_ik(
        self,
        left_wrist: np.ndarray,
        right_wrist: np.ndarray,
        q_init: np.ndarray | None = None,
        max_iters: int = 5,
        pos_threshold: float = 1e-4,
        ori_threshold: float = 1e-4,
        verbose: bool = True,
    ) -> np.ndarray:
        """求解双臂逆运动学。

        Parameters
        ----------
        left_wrist : np.ndarray
            左手腕目标位姿 (4x4 齐次变换矩阵)。
        right_wrist : np.ndarray
            右手腕目标位姿 (4x4 齐次变换矩阵)。
        q_init : np.ndarray, optional
            初始关节角度（热启动）。默认为零位。
        max_iters : int
            最大外部迭代次数。
        pos_threshold : float
            位置误差阈值 (m)。
        ori_threshold : float
            角度误差阈值 (rad)。
        verbose : bool
            是否打印每次迭代的误差信息。

        Returns
        -------
        np.ndarray
            求解后的完整关节角度向量（简化模型）。
        """
        if q_init is not None:
            self.init_data = q_init

        for i in range(max_iters):
            try:
                t0 = time.time()

                arg_l = casadi.DM(left_wrist)
                arg_r = casadi.DM(right_wrist)
                arg_q = casadi.DM(self.init_data)

                sol_q_dm = self.ik_solver_func(arg_l, arg_r, arg_q)
                sol_q = np.array(sol_q_dm).flatten()

                # 计算当前误差
                placement_L = self.reduced_robot.framePlacement(sol_q, self.L_hand_id)
                placement_R = self.reduced_robot.framePlacement(sol_q, self.R_hand_id)

                dist_L = float(np.linalg.norm(placement_L.translation - left_wrist[:3, 3]))
                dist_R = float(np.linalg.norm(placement_R.translation - right_wrist[:3, 3]))

                rot_err_L = float(np.linalg.norm(
                    pin.log3(placement_L.rotation.T @ left_wrist[:3, :3])))
                rot_err_R = float(np.linalg.norm(
                    pin.log3(placement_R.rotation.T @ right_wrist[:3, :3])))

                self.init_data = sol_q

                elapsed = (time.time() - t0) * 1000
                if verbose:
                    print(
                        f"IK solve: {elapsed:6.2f} ms | "
                        f"iter: {i + 1}/{max_iters} | "
                        f"dist L:{dist_L * 1000:.3f}mm R:{dist_R * 1000:.3f}mm | "
                        f"rot err L:{rot_err_L:.4f} R:{rot_err_R:.4f}"
                    )

                if max(dist_L, dist_R) < pos_threshold and max(rot_err_L, rot_err_R) < ori_threshold:
                    break

            except Exception as exc:
                print(f"[WalkerArmIK] 第 {i} 次迭代失败: {exc}")
                break

        return self.init_data

    def fk(
        self,
        q: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        """计算给定配置下左右手腕的正向运动学位姿。

        Parameters
        ----------
        q : np.ndarray
            简化模型的关节角度。

        Returns
        -------
        tuple[np.ndarray, np.ndarray]
            (left_pose, right_pose)，各为 4x4 齐次变换矩阵。
        """
        placement_L = self.reduced_robot.framePlacement(q, self.L_hand_id)
        placement_R = self.reduced_robot.framePlacement(q, self.R_hand_id)
        return placement_L.homogeneous, placement_R.homogeneous

    # ------------------------------------------------------------------
    # 代码生成
    # ------------------------------------------------------------------

    def export_so(
        self,
        output_so: str = "ik_solver.so",
        conda_prefix: str | None = None,
    ) -> None:
        """将 IK 求解器编译为 .so 动态库。

        Parameters
        ----------
        output_so : str
            输出 .so 文件路径。
        conda_prefix : str, optional
            Conda 环境路径。默认使用 CONDA_PREFIX 环境变量。
        """
        if conda_prefix is None:
            conda_prefix = os.environ.get("CONDA_PREFIX", "")
        if not conda_prefix:
            raise RuntimeError("请设置 CONDA_PREFIX 或传入 conda_prefix 参数")

        c_file = "ik_solver.c"
        print("[WalkerArmIK] 生成 C 代码...")
        cg = casadi.CodeGenerator(c_file)
        f_expanded = self.ik_solver_func.expand()
        cg.add(f_expanded)
        cg.generate()

        casadi_path = os.path.dirname(casadi.__file__)

        inc_paths = [f"-I{conda_prefix}/include", f"-I{casadi_path}/include"]
        lib_paths = [f"-L{casadi_path}", f"-L{conda_prefix}/lib"]
        libs = ["-lcasadi", "-lipopt", "-lstdc++", "-lm"]
        rpath = f"-Wl,-rpath,{casadi_path}:{conda_prefix}/lib"

        comp_cmd = (
            f"gcc -fPIC -shared -O3 {c_file} -Dcasadi_inf=INFINITY "
            f"-o {output_so} {' '.join(inc_paths)} {' '.join(lib_paths)} "
            f"{rpath} {' '.join(libs)}"
        )

        print(f"[WalkerArmIK] 编译: {comp_cmd}")
        ret = os.system(comp_cmd)
        if ret == 0:
            print(f"[WalkerArmIK] 成功生成 {output_so}")
        else:
            print("[WalkerArmIK] 编译失败。")
