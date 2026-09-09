#!/usr/bin/env python3
"""Jetson 板卡只读验收工具。"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import platform
import shutil
import subprocess
from typing import Iterable


@dataclass(frozen=True)
class CheckResult:
    """保存一项验收结果。"""

    name: str
    status: str
    detail: str


def read_text(path: Path) -> str:
    """读取文本文件，并清除设备树字符串末尾的空字符。"""
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
        return text.replace("\x00", "").strip()
    except (OSError, TypeError):
        return ""


def run_command(command: list[str], timeout_sec: float = 8.0) -> str:
    """执行只读命令；命令不存在或失败时返回可读提示。"""
    if shutil.which(command[0]) is None:
        return f"命令不存在：{command[0]}"
    try:
        result = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout_sec,
        )
    except subprocess.TimeoutExpired:
        return f"命令超时：{' '.join(command)}"

    output_parts = (result.stdout, result.stderr)
    output = "\n".join(part.strip() for part in output_parts if part.strip())
    if output:
        return output
    return f"命令退出码：{result.returncode}，没有输出"


def command_path(command: str, preferred_path: str) -> str:
    """优先使用 Jetson 上的标准绝对路径。"""
    if Path(preferred_path).is_file():
        return preferred_path
    return command


def find_network_interfaces() -> tuple[list[str], list[str]]:
    """从 sysfs 区分无线网口和物理有线网口。"""
    network_root = Path("/sys/class/net")
    wireless: list[str] = []
    ethernet: list[str] = []
    if not network_root.exists():
        return wireless, ethernet

    for interface_path in sorted(network_root.iterdir()):
        name = interface_path.name
        if name == "lo":
            continue
        if (interface_path / "wireless").exists():
            wireless.append(name)
            continue
        if (interface_path / "device").exists():
            ethernet.append(name)
    return wireless, ethernet


def interface_driver(interface: str) -> str:
    """读取网口当前绑定的内核驱动名称。"""
    driver_path = Path("/sys/class/net") / interface / "device" / "driver"
    try:
        return driver_path.resolve(strict=True).name
    except (OSError, TypeError):
        return "未找到已绑定驱动"


def read_temperatures() -> list[tuple[str, float]]:
    """读取所有可用温度区，单位转换为摄氏度。"""
    temperatures: list[tuple[str, float]] = []
    for zone in sorted(Path("/sys/class/thermal").glob("thermal_zone*")):
        raw_value = read_text(zone / "temp")
        if not raw_value:
            continue
        try:
            value = float(raw_value)
        except ValueError:
            continue
        if value > 1000.0:
            value /= 1000.0
        label = read_text(zone / "type") or zone.name
        temperatures.append((label, value))
    return temperatures


def format_gib(byte_count: int) -> str:
    """把字节数转换为 GiB。"""
    return f"{byte_count / (1024 ** 3):.1f} GiB"


def build_checks() -> tuple[list[CheckResult], list[tuple[str, str]]]:
    """执行板卡验收并返回结论与只读命令明细。"""
    checks: list[CheckResult] = []
    details: list[tuple[str, str]] = []

    model = read_text(Path("/proc/device-tree/model"))
    if "jetson" in model.lower():
        checks.append(CheckResult("Jetson 型号", "通过", model))
    else:
        checks.append(CheckResult("Jetson 型号", "失败", model or "未读取到设备树型号"))

    architecture = platform.machine()
    architecture_status = "通过" if architecture == "aarch64" else "失败"
    checks.append(CheckResult("CPU 架构", architecture_status, architecture))

    l4t_version = read_text(Path("/etc/nv_tegra_release"))
    checks.append(
        CheckResult(
            "Jetson Linux/L4T",
            "通过" if l4t_version else "警告",
            l4t_version or "没有找到 /etc/nv_tegra_release",
        )
    )
    checks.append(CheckResult("Linux 内核", "通过", platform.release()))

    nvme_devices = sorted(
        path.name for path in Path("/sys/block").glob("nvme*n*")
    )
    checks.append(
        CheckResult(
            "NVMe 固态硬盘",
            "通过" if nvme_devices else "警告",
            ", ".join(nvme_devices) if nvme_devices else "系统没有枚举到 NVMe 设备",
        )
    )

    disk = shutil.disk_usage("/")
    free_status = "通过" if disk.free >= 30 * 1024 ** 3 else "警告"
    checks.append(
        CheckResult(
            "根分区可用空间",
            free_status,
            f"可用 {format_gib(disk.free)}，总计 {format_gib(disk.total)}",
        )
    )

    pci_devices = run_command(["lspci", "-nnk"])
    if "8822" in pci_devices.lower() or "10ec:c822" in pci_devices.lower():
        checks.append(CheckResult("RTL8822CE 枚举", "通过", "PCIe 已发现 RTL8822CE"))
    else:
        checks.append(
            CheckResult(
                "RTL8822CE 枚举",
                "警告",
                "lspci 输出中未找到 8822CE；请检查驱动、插槽和固定情况",
            )
        )

    wireless, ethernet = find_network_interfaces()
    if wireless:
        driver_text = ", ".join(
            f"{interface}（驱动 {interface_driver(interface)}）"
            for interface in wireless
        )
        checks.append(CheckResult("无线网口", "通过", driver_text))
    else:
        checks.append(CheckResult("无线网口", "失败", "未发现无线网口"))

    if ethernet:
        driver_text = ", ".join(
            f"{interface}（驱动 {interface_driver(interface)}）"
            for interface in ethernet
        )
        checks.append(CheckResult("有线网口", "通过", driver_text))
    else:
        checks.append(CheckResult("有线网口", "失败", "未发现物理有线网口"))

    temperatures = read_temperatures()
    if temperatures:
        hottest_name, hottest_value = max(
            temperatures, key=lambda item: item[1]
        )
        temperature_status = "通过" if hottest_value < 80.0 else "警告"
        checks.append(
            CheckResult(
                "板卡温度",
                temperature_status,
                f"最高 {hottest_value:.1f} °C（{hottest_name}）",
            )
        )
    else:
        checks.append(CheckResult("板卡温度", "警告", "没有读取到温度传感器"))

    details.extend(
        [
            ("网络设备状态", run_command(["nmcli", "device", "status"])),
            ("无线射频开关", run_command(["rfkill", "list"])),
            ("PCIe 设备和驱动", pci_devices),
            (
                "磁盘与挂载点",
                run_command(
                    ["lsblk", "-o", "NAME,SIZE,TYPE,FSTYPE,MOUNTPOINTS,MODEL"]
                ),
            ),
            (
                "Jetson 功耗模式",
                run_command(
                    [
                        command_path("nvpmodel", "/usr/sbin/nvpmodel"),
                        "-q",
                        "--verbose",
                    ]
                ),
            ),
            (
                "Jetson 短时资源与温度",
                run_command(
                    [
                        command_path("tegrastats", "/usr/bin/tegrastats"),
                        "--interval",
                        "500",
                        "--count",
                        "3",
                    ]
                ),
            ),
        ]
    )
    return checks, details


def render_report(
    checks: Iterable[CheckResult], details: Iterable[tuple[str, str]]
) -> str:
    """生成便于现场保存和回传的中文报告。"""
    check_list = list(checks)
    failure_count = sum(item.status == "失败" for item in check_list)
    warning_count = sum(item.status == "警告" for item in check_list)
    if failure_count:
        overall = f"失败（{failure_count} 项失败，{warning_count} 项警告）"
    elif warning_count:
        overall = f"有警告（{warning_count} 项需要复核）"
    else:
        overall = "通过"

    lines = [
        "Jetson 板卡只读验收报告",
        f"生成时间：{datetime.now().astimezone().isoformat(timespec='seconds')}",
        "说明：本程序没有修改网络、功耗模式或任何系统配置。",
        f"总体结论：{overall}",
        "",
        "一、结论",
    ]
    for item in check_list:
        lines.append(f"[{item.status}] {item.name}：{item.detail}")

    lines.extend(["", "二、原始信息"])
    for title, output in details:
        lines.extend(["", f"--- {title} ---", output])
    return "\n".join(lines) + "\n"


def parse_args() -> argparse.Namespace:
    """解析命令行参数。"""
    parser = argparse.ArgumentParser(description="只读检查 Jetson 板卡硬件和系统状态")
    parser.add_argument(
        "--output",
        default="jetson_board_report.txt",
        help="报告保存路径，默认保存在当前目录",
    )
    return parser.parse_args()


def main() -> None:
    """程序入口。"""
    args = parse_args()
    checks, details = build_checks()
    report = render_report(checks, details)
    print(report, end="")

    output_path = Path(args.output).expanduser().resolve()
    try:
        output_path.write_text(report, encoding="utf-8")
    except OSError as error:
        print(f"[警告] 报告保存失败：{error}")
    else:
        print(f"报告已保存：{output_path}")


if __name__ == "__main__":
    main()
