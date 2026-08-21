# UBT Robot Python SDK wheels

本目录保存 UBT Robotics 提供的 `ubt_robot` Python SDK 预编译 wheel，供
`box_grasp_demo` 离线构建使用。它们是随源码树一同交付的 vendor 二进制文件，
构建过程不会从网络下载 SDK，也不会使用 `pip` 或写入系统 Python。wheel 元数据
标识作者为 UBT Robotics、版本为 1.0.0。公司已确认这些 SDK 文件可以随本项目
发布；它们受项目 `doc/LICENSES/UBTECH-PROPRIETARY.txt` 约束，仅允许使用。
除协议明确授权的安装期 RUNPATH 调整外，禁止修改；禁止在 GitHub 之外分发。
SDK 第三方组件及对应许可证见
`doc/LICENSES/SDK-THIRD-PARTY-NOTICES.md`。

## 支持范围

| wheel 平台标签 | `platform.machine()` | CPU 架构 |
| --- | --- | --- |
| `linux_x86_64` | `x86_64`、`amd64` | AMD64 / Intel 64 |
| `linux_aarch64` | `aarch64`、`arm64` | ARM64 |

这些 wheel 只用于 Linux CPython 3.10，ABI 标签必须为
`cp310-cp310`。安装脚本也必须由 CPython 3.10 执行。SDK 会安装到指定 ROS
install prefix 的 `lib/python3.10/site-packages`，不会安装到 `local/`。

典型的构建期调用如下：

```sh
/usr/bin/python3.10 cmake/install_ubt_robot_wheel.py \
  --wheel-dir vendor/ubt_robot \
  --prefix /path/to/ros/install/box_grasp_demo
```

安装脚本根据当前 CPU 精确选择一份 wheel，使用 Python 标准库 `zipfile` 解包，
并检查归档路径、`ubt_robot/__init__.py` 和 `_core*.so`。重复执行会覆盖 wheel
中同名的已安装文件。

安装器还会校验供应商原始 wheel 的固定 SHA-256。原件不会被改写；安装副本中的
`libcc_api_client.so*` 会把供应商构建机绝对 RUNPATH（含尾部空路径项）收紧为
`$ORIGIN`，避免从当前工作目录加载共享库。供应商 wheel 的内部平台元数据不规范，且
元数据中的 `License: UNKNOWN` 不代表这些文件没有许可；适用条款以项目
`doc/LICENSES/UBTECH-PROPRIETARY.txt` 和
`doc/LICENSES/SDK-THIRD-PARTY-NOTICES.md` 为准。

## 系统依赖

wheel 自带 `libcc_api_client` 和相关 `libtbox` 共享库，但运行时仍依赖目标 Linux
系统提供兼容的 glibc、`libstdc++.so.6`、`libgcc_s.so.1` 和
`libmosquitto.so.1`（Ubuntu 通常由 `libmosquitto1` 软件包提供）。

`libmosquitto.so.1` 的存在性由项目 CMake 配置检查；离线安装脚本不会检查、下载
或安装该系统库。如果系统依赖缺失，文件仍可正确安装，但 `import ubt_robot` 可能
在动态加载 `_core` 时失败。
