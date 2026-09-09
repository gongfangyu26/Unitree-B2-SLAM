"""宇树 B2 Sport 接口编号和安全白名单测试."""

import pytest

from b2_twist_bridge_py.sport_model import ROBOT_SPORT_API_IDS, api_id


def test_required_b2_api_ids_match_official_b2_interface():
    """巡检直接使用的接口编号必须与宇树 B2 专用头文件一致."""
    assert api_id("STOPMOVE") == 1003
    assert api_id("STANDUP") == 1004
    assert api_id("STANDDOWN") == 1005
    assert api_id("MOVE") == 1008


def test_go2_sit_alias_is_not_exposed_for_b2():
    """B2 专用编号表不能沿用 Go2 的坐下动作名称."""
    assert "SIT" not in ROBOT_SPORT_API_IDS


def test_b2_specific_advanced_ids_match_official_example():
    """抽查容易与 A2 或 Go2 混淆的 B2 专用高级编号."""
    assert ROBOT_SPORT_API_IDS["TRAJECTORYFOLLOW"] == 1018
    assert ROBOT_SPORT_API_IDS["HANDSTAND"] == 1039
    assert ROBOT_SPORT_API_IDS["FREEEULER"] == 1051
    assert ROBOT_SPORT_API_IDS["VISIONWALK"] == 1101


def test_dangerous_action_is_blocked_by_direct_api():
    """基础测试程序不能直接调用倒立等危险动作."""
    with pytest.raises(ValueError, match="禁止"):
        api_id("HANDSTAND")
