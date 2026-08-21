#!/usr/bin/env python3
"""Install the vendored ubt_robot wheel into a ROS install prefix."""

from __future__ import annotations

import argparse
import hashlib
import os
import platform
import shutil
import stat
import sys
import tempfile
import zipfile
from pathlib import Path, PurePosixPath
from typing import Optional


PYTHON_VERSION = (3, 10)
SITE_PACKAGES_RELATIVE = Path("lib/python3.10/site-packages")
ARCHITECTURES = {
    "x86_64": "x86_64",
    "amd64": "x86_64",
    "aarch64": "aarch64",
    "arm64": "aarch64",
}
EXPECTED_WHEEL_SHA256 = {
    "aarch64": "40e5ae65e82caaf05845d5ffdeae21385a5505281f6662e5a925030ed28fc3fc",
    "x86_64": "fb86e24787378189927b8b5392df5cd101b51f6612fd64ae0be2847af340ee31",
}
UNSAFE_VENDOR_RUNPATH = (
    b"/opt/ubt_3rdparty/mosquitto/lib:"
    b"/opt/ubt_3rdparty/nlohmann_json/lib:"
    b"/opt/walker/cc_api_client/cpp-tbox_vendor/share/"
    b"cpp-tbox_vendor/cmake/../../../lib:"
)


class InstallError(RuntimeError):
    """An expected validation or installation failure."""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Install the matching vendored ubt_robot CPython 3.10 wheel "
            "without invoking pip or accessing the network."
        )
    )
    parser.add_argument(
        "--wheel-dir",
        required=True,
        type=Path,
        help="directory containing the architecture-specific ubt_robot wheels",
    )
    parser.add_argument(
        "--prefix",
        required=True,
        type=Path,
        help="ROS install prefix (files go under lib/python3.10/site-packages)",
    )
    return parser.parse_args()


def require_python_310() -> None:
    implementation = platform.python_implementation()
    if implementation != "CPython" or sys.version_info[:2] != PYTHON_VERSION:
        actual = f"{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}"
        raise InstallError(
            "ubt_robot requires the CPython 3.10 ABI, but this installer is "
            f"running under {implementation} {actual}. Invoke it with a CPython 3.10 "
            "interpreter (for example, /usr/bin/python3.10)."
        )


def current_architecture() -> str:
    machine = platform.machine().strip().lower()
    try:
        return ARCHITECTURES[machine]
    except KeyError as exc:
        supported = ", ".join(sorted(ARCHITECTURES))
        raise InstallError(
            f"unsupported CPU architecture {machine!r}; supported platform.machine() "
            f"values are: {supported}"
        ) from exc


def select_wheel(wheel_dir: Path, architecture: str) -> Path:
    if not wheel_dir.is_dir():
        raise InstallError(f"wheel directory does not exist or is not a directory: {wheel_dir}")

    pattern = f"ubt_robot-*-cp310-cp310-linux_{architecture}.whl"
    matches = sorted(path for path in wheel_dir.glob(pattern) if path.is_file())
    if len(matches) != 1:
        rendered = ", ".join(path.name for path in matches) or "none"
        raise InstallError(
            f"expected exactly one wheel matching {pattern!r} in {wheel_dir}, "
            f"found {len(matches)}: {rendered}"
        )
    return matches[0]


def verify_wheel_sha256(wheel: Path, architecture: str) -> None:
    digest = hashlib.sha256()
    with wheel.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    actual = digest.hexdigest()
    expected = EXPECTED_WHEEL_SHA256[architecture]
    if actual != expected:
        raise InstallError(
            f"wheel SHA-256 mismatch for {wheel.name}: "
            f"expected {expected}, got {actual}")


def normalized_member_path(name: str) -> PurePosixPath:
    """Return a safe, relative wheel member path."""
    if not name or "\x00" in name:
        raise InstallError(f"wheel contains an invalid member name: {name!r}")
    if "\\" in name:
        raise InstallError(f"wheel member uses a non-POSIX path separator: {name!r}")

    path_text = name[:-1] if name.endswith("/") else name
    raw_parts = path_text.split("/")
    if not path_text or any(part in ("", ".", "..") for part in raw_parts):
        raise InstallError(f"wheel contains an unsafe member path: {name!r}")

    relative = PurePosixPath(path_text)
    if relative.is_absolute():
        raise InstallError(f"wheel contains an absolute member path: {name!r}")
    return relative


def installation_relative_path(
    member: PurePosixPath, *, is_directory: bool
) -> Optional[PurePosixPath]:
    """Map wheel roots and purelib/platlib data schemes into site-packages."""
    parts = member.parts
    if not parts[0].endswith(".data"):
        return member

    if len(parts) == 1 and is_directory:
        return None
    if len(parts) < 2:
        raise InstallError(f"invalid wheel .data member: {member}")

    scheme = parts[1]
    if scheme not in ("purelib", "platlib"):
        raise InstallError(
            f"unsupported wheel .data scheme {scheme!r} in member {member}; "
            "this installer accepts only purelib and platlib"
        )
    if len(parts) == 2:
        if is_directory:
            return None
        raise InstallError(f"invalid wheel .data member without a destination name: {member}")
    return PurePosixPath(*parts[2:])


def checked_destination(root: Path, relative: PurePosixPath) -> Path:
    """Resolve existing symlinks and ensure a destination cannot escape root."""
    resolved_root = root.resolve(strict=False)
    destination = root.joinpath(*relative.parts)
    resolved_destination = destination.resolve(strict=False)
    try:
        resolved_destination.relative_to(resolved_root)
    except ValueError as exc:
        raise InstallError(
            f"wheel member destination escapes site-packages: {relative}"
        ) from exc
    return destination


def validate_member_type(info: zipfile.ZipInfo) -> None:
    unix_mode = info.external_attr >> 16
    file_type = stat.S_IFMT(unix_mode)
    expected_type = stat.S_IFDIR if info.is_dir() else stat.S_IFREG
    if file_type not in (0, expected_type):
        raise InstallError(
            f"wheel member is not a regular file or directory: {info.filename!r}"
        )


def installation_plan(
    archive: zipfile.ZipFile, site_packages: Path
) -> list[tuple[zipfile.ZipInfo, Optional[Path], Optional[PurePosixPath]]]:
    plan: list[tuple[zipfile.ZipInfo, Optional[Path], Optional[PurePosixPath]]] = []
    destinations: dict[Path, tuple[str, bool]] = {}
    package_files: set[PurePosixPath] = set()

    for info in archive.infolist():
        validate_member_type(info)
        member = normalized_member_path(info.filename)
        relative = installation_relative_path(member, is_directory=info.is_dir())
        destination = None if relative is None else checked_destination(site_packages, relative)

        if destination is not None:
            destination_key = destination.resolve(strict=False)
            previous = destinations.get(destination_key)
            if previous is not None and not (previous[1] and info.is_dir()):
                raise InstallError(
                    f"wheel members {previous[0]!r} and {info.filename!r} map to the "
                    f"same destination: {destination}"
                )
            destinations[destination_key] = (info.filename, info.is_dir())

            if not info.is_dir():
                package_files.add(relative)

        plan.append((info, destination, relative))

    resolved_root = site_packages.resolve(strict=False)
    file_destinations = {
        destination: source
        for destination, (source, is_directory) in destinations.items()
        if not is_directory
    }
    for destination, (source, _is_directory) in destinations.items():
        parent = destination.parent
        while parent != resolved_root:
            conflicting_source = file_destinations.get(parent)
            if conflicting_source is not None:
                raise InstallError(
                    f"wheel member {source!r} would be installed below file member "
                    f"{conflicting_source!r}"
                )
            parent = parent.parent

    initializer = PurePosixPath("ubt_robot/__init__.py")
    if initializer not in package_files:
        raise InstallError("wheel is missing required file ubt_robot/__init__.py")

    extension_modules = sorted(
        path
        for path in package_files
        if len(path.parts) == 2
        and path.parts[0] == "ubt_robot"
        and path.name.startswith("_core")
        and path.name.endswith(".so")
    )
    if not extension_modules:
        raise InstallError("wheel is missing required extension ubt_robot/_core*.so")

    return plan


def install_file(archive: zipfile.ZipFile, info: zipfile.ZipInfo, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary_name: Optional[str] = None
    try:
        with archive.open(info, "r") as source, tempfile.NamedTemporaryFile(
            mode="wb",
            prefix=f".{destination.name}.",
            dir=destination.parent,
            delete=False,
        ) as temporary:
            temporary_name = temporary.name
            shutil.copyfileobj(source, temporary)

        unix_mode = info.external_attr >> 16
        permissions = stat.S_IMODE(unix_mode) or 0o644
        os.chmod(temporary_name, permissions)
        os.replace(temporary_name, destination)
        temporary_name = None
    finally:
        if temporary_name is not None:
            try:
                os.unlink(temporary_name)
            except FileNotFoundError:
                pass


def install_wheel(wheel: Path, site_packages: Path) -> int:
    try:
        with zipfile.ZipFile(wheel, "r") as archive:
            corrupt_member = archive.testzip()
            if corrupt_member is not None:
                raise InstallError(
                    f"wheel failed its CRC check at member {corrupt_member!r}: {wheel}"
                )
            plan = installation_plan(archive, site_packages)
            site_packages.mkdir(parents=True, exist_ok=True)

            installed_files = 0
            for info, destination, _relative in plan:
                if destination is None:
                    continue
                if info.is_dir():
                    destination.mkdir(parents=True, exist_ok=True)
                else:
                    install_file(archive, info, destination)
                    installed_files += 1
            return installed_files
    except InstallError:
        raise
    except (OSError, RuntimeError, zipfile.BadZipFile, zipfile.LargeZipFile) as exc:
        raise InstallError(f"failed to install wheel {wheel}: {exc}") from exc


def harden_installed_runpaths(site_packages: Path) -> None:
    """Replace the vendor build-host RUNPATH in installed SDK libraries.

    The original wheel is retained byte-for-byte and verified by SHA-256.  Its
    installed libcc_api_client copies contain absolute build-machine paths and
    a trailing empty entry (current-directory lookup).  Replacing that string
    in-place with ``$ORIGIN`` keeps the ELF string-table layout unchanged while
    restricting dependency lookup to the installed package directory.
    """
    package = site_packages / "ubt_robot"
    candidates = sorted(package.glob("libcc_api_client.so*"))
    if not candidates:
        raise InstallError("installed wheel is missing libcc_api_client.so*")
    replacement = b"$ORIGIN\0" + b"\0" * (
        len(UNSAFE_VENDOR_RUNPATH) - len(b"$ORIGIN\0"))
    for library in candidates:
        data = library.read_bytes()
        count = data.count(UNSAFE_VENDOR_RUNPATH)
        if count != 1:
            raise InstallError(
                f"expected one known vendor RUNPATH in {library.name}, found {count}")
        hardened = data.replace(UNSAFE_VENDOR_RUNPATH, replacement, 1)
        if UNSAFE_VENDOR_RUNPATH in hardened:
            raise InstallError(f"failed to remove unsafe RUNPATH from {library.name}")
        library.write_bytes(hardened)


def main() -> int:
    args = parse_args()
    try:
        require_python_310()
        architecture = current_architecture()
        wheel = select_wheel(args.wheel_dir.resolve(), architecture)
        verify_wheel_sha256(wheel, architecture)
        site_packages = args.prefix.resolve(strict=False) / SITE_PACKAGES_RELATIVE
        installed_files = install_wheel(wheel, site_packages)
        harden_installed_runpaths(site_packages)
    except InstallError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    except OSError as exc:
        print(f"error: filesystem operation failed: {exc}", file=sys.stderr)
        return 1

    print(
        f"Installed {installed_files} files from {wheel.name} into {site_packages} "
        f"for CPython 3.10/{architecture}."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
