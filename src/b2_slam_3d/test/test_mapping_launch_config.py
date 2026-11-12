"""防止三维导出选项被启动文件的二维默认覆盖。"""

import ast
from pathlib import Path


LAUNCH = Path(__file__).resolve().parents[1] / "launch" / "slam_3d.launch.py"


def call(node, name):
    """识别启动配置中的函数调用。"""
    return isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == name


def test_grid_export_options_are_strings_and_reach_both_database_modes():
    """新建/继续数据库两种模式都必须采用同一显式三维配置。"""
    tree = ast.parse(LAUNCH.read_text(encoding="utf-8"))
    defaults = {
        node.args[0].value: next(k.value.value for k in node.keywords if k.arg == "default_value")
        for node in ast.walk(tree) if call(node, "DeclareLaunchArgument")
        and any(k.arg == "default_value" and isinstance(k.value, ast.Constant)
                for k in node.keywords)
    }
    assert defaults["grid_3d"] == "false"  # 原导航基线默认不变
    assert defaults["grid_max_obstacle_height"] == "1.0"
    assert defaults["grid_ray_tracing"] == "true"
    grid = next(
        node.value for node in ast.walk(tree) if isinstance(node, ast.Assign)
        and any(isinstance(t, ast.Name) and t.id == "grid_parameters" for t in node.targets)
    )
    parameters = {k.value: v for k, v in zip(grid.keys, grid.values)}
    for key, argument in (
        ("Grid/3D", "grid_3d"),
        ("Grid/MaxObstacleHeight", "grid_max_obstacle_height"),
        ("Grid/RayTracing", "grid_ray_tracing"),
    ):
        value = parameters[key]
        assert call(value, "ParameterValue")
        assert call(value.args[0], "LaunchConfiguration")
        assert value.args[0].args[0].value == argument
        assert any(k.arg == "value_type" and isinstance(k.value, ast.Name)
                   and k.value.id == "str" for k in value.keywords)
    nodes = [
        node for node in ast.walk(tree) if call(node, "Node")
        and any(k.arg == "package" and isinstance(k.value, ast.Constant)
                and k.value.value == "rtabmap_slam" for k in node.keywords)
    ]
    assert len(nodes) == 2
    for node in nodes:
        values = next(k.value for k in node.keywords if k.arg == "parameters")
        assert isinstance(values.elts[1], ast.Name) and values.elts[1].id == "grid_parameters"
