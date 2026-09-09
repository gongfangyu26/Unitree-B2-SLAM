"""
宇树 B2 Sport 服务接口编号.

编号依据宇树官方 ``unitree_ros2`` 的 B2 专用
``example/src/include/common/ros2_b2_sport_client.h``。这里不沿用 Go2 或 A2
的同名编号表，避免高级动作在不同机型之间被错误解释。
"""


ROBOT_SPORT_API_IDS = {
    "DAMP": 1001,  # 阻尼控制
    "BALANCESTAND": 1002,  # 平衡站立
    "STOPMOVE": 1003,  # 停止运动
    "STANDUP": 1004,  # 站立起身
    "STANDDOWN": 1005,  # 趴下/降低机身
    "RECOVERYSTAND": 1006,  # 摔倒后恢复站立
    "MOVE": 1008,  # 机身速度控制
    "SWITCHGAIT": 1011,  # 切换步态
    "BODYHEIGHT": 1013,  # 机身高度控制
    "SPEEDLEVEL": 1015,  # 速度等级
    "TRAJECTORYFOLLOW": 1018,  # 轨迹跟随
    "CONTINUOUSGAIT": 1019,  # 连续步态
    "MOVETOPOS": 1036,  # 移动到机身坐标位置
    "SWITCHMOVEMODE": 1038,  # 切换运动模式
    "HANDSTAND": 1039,  # 倒立，危险动作，项目主流程禁止调用
    "AUTORECOVERY_SET": 1040,  # 设置自动恢复
    "FREEWALK": 1045,  # 自由行走模式
    "CLASSICWALK": 1049,  # 经典行走模式
    "FASTWALK": 1050,  # 快速行走模式
    "FREEEULER": 1051,  # 自由欧拉角控制
    "VISIONWALK": 1101,  # 视觉行走模式
}


# 项目内允许直接发送的低风险动作白名单。危险动作即使在上表中有官方编号，
# 也不会被基础测试程序或速度桥调用。
SAFE_DIRECT_COMMANDS = frozenset(
    {
        "DAMP",
        "BALANCESTAND",
        "STOPMOVE",
        "STANDUP",
        "STANDDOWN",
        "RECOVERYSTAND",
        "MOVE",
    }
)


def api_id(command: str) -> int:
    """返回经过白名单检查的基础控制接口编号."""
    if command not in SAFE_DIRECT_COMMANDS:
        raise ValueError(f"基础控制禁止发送动作：{command}")
    return ROBOT_SPORT_API_IDS[command]
