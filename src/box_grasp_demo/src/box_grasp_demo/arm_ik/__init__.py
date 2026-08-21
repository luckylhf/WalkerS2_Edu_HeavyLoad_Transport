"""
Walker S2 双臂逆运动学 (IK) 求解器

基于 Pinocchio 运动学库和 CasADi 优化框架，使用 IPOPT 非线性求解器。
支持代码生成（.so 导出）以加速实时求解。

用法:
    from box_grasp_demo.arm_ik import WalkerArmIK

    ik = WalkerArmIK(urdf_path="path/to/walker_s2.urdf")
    sol_q = ik.solve_ik(left_target, right_target)

    # 或者加载预编译的 .so 文件：
    ik = WalkerArmIK(urdf_path="path/to/walker_s2.urdf", so_path="ik_solver.so")
"""

from .walker_ik import WalkerArmIK

__all__ = ["WalkerArmIK"]
