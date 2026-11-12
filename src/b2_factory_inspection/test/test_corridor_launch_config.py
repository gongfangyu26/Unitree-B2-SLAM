"""离线检查走廊启动配置与速度桥、DWB 的参数连通性。"""

import ast
from pathlib import Path

from b2_twist_bridge_py.velocity_policy import VelocityPolicy


ROOT = Path(__file__).resolve().parents[2]


def source_tree(package, filename):
    """解析源码，不执行 launch 或创建 DDS 节点。"""
    return ast.parse((ROOT / package / "launch" / filename).read_text(encoding="utf-8"))


def is_call(node, name):
    """判断一个 AST 调用的简单函数名。"""
    return isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == name


def dict_value(node, key):
    """读取 launch 字典中的一个显式键。"""
    for item_key, value in zip(node.keys, node.values):
        if isinstance(item_key, ast.Constant) and item_key.value == key:
            return value
    raise AssertionError(f"缺少启动配置键 {key}")


def keyword(node, name):
    """读取一个调用的关键字参数。"""
    return next(item.value for item in node.keywords if item.arg == name)


def test_corridor_speed_and_duration_defaults_reach_navigation():
    """不能出现只提高时间/文档，却没给导航链传递目标速度的情况。"""
    tree = source_tree("b2_factory_inspection", "corridor_patrol.launch.py")
    defaults = {
        node.args[0].value: ast.literal_eval(keyword(node, "default_value"))
        for node in ast.walk(tree) if is_call(node, "DeclareLaunchArgument")
        and any(item.arg == "default_value" for item in node.keywords)
    }
    assert defaults["laps"] == "1"
    assert defaults["max_linear_x"] == "0.60"
    assert defaults["goal_timeout_sec"] == "60.0"
    assert defaults["mission_timeout_sec"] == "600.0"
    include = next(node for node in ast.walk(tree) if is_call(node, "IncludeLaunchDescription"))
    arguments = keyword(include, "launch_arguments").func.value
    for key in ("max_linear_x", "mission_timeout_sec"):
        argument = "max_armed_duration_sec" if key == "mission_timeout_sec" else key
        value = dict_value(arguments, argument)
        assert is_call(value, "LaunchConfiguration") and value.args[0].value == key


def test_same_speed_argument_controls_both_dwb_limits_and_bridge():
    """DWB 及 driver 必须读同一个 max_linear_x，不能被静态 0.30 配置截断。"""
    tree = source_tree("b2_navigation", "navigation.launch.py")
    controller = next(
        node for node in ast.walk(tree) if is_call(node, "Node")
        and ast.literal_eval(keyword(node, "name")) == "controller_server"
    )
    override = keyword(controller, "parameters").elts[-1]
    for key in ("FollowPath.max_vel_x", "FollowPath.max_speed_xy"):
        value = dict_value(override, key)
        assert is_call(value, "ParameterValue")
        assert isinstance(value.args[0], ast.Name) and value.args[0].id == "max_linear_x"
    driver = next(
        node for node in ast.walk(tree) if is_call(node, "IncludeLaunchDescription")
    )
    value = dict_value(keyword(driver, "launch_arguments").func.value, "max_linear_x")
    assert isinstance(value, ast.Name) and value.id == "max_linear_x"


def test_six_tenths_ceiling_preserves_slowing_and_zero_stop():
    """提高上限不能把每个非零命令强制为 0.60，也不能破坏零速停车。"""
    policy = VelocityPolicy(max_linear_x=0.60)
    assert policy.shape(0.60, 0.0, 0.0)[:3] == (0.60, 0.0, 0.0)
    assert policy.shape(0.90, 0.0, 0.0)[:3] == (0.60, 0.0, 0.0)
    assert policy.shape(0.40, 0.0, 0.0)[:3] == (0.40, 0.0, 0.0)
    assert policy.shape(0.08, 0.0, 0.0)[:3] == (0.30, 0.0, 0.0)
    assert policy.shape(0.0, 0.0, 0.0)[:3] == (0.0, 0.0, 0.0)
    assert policy.shape(0.0, 0.0, 1.0)[:3] == (0.0, 0.0, 0.35)
