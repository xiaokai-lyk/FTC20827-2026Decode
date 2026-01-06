import math

def wheel_diameter_from_motor_point(
    motor_free_rpm: float,
    target_motor_rpm: float = None,
    target_ratio: float = None,
    gear_ratio: float = 1.0,
    target_linear_speed_mps: float = None,
    reference_wheel_diameter_mm: float = None,
    reference_wheel_rpm: float = None,
):
    """
    计算在给定减速比下，使电机工作在目标转速所需的轮子直径

    参数说明：
    motor_free_rpm              电机空载转速 (RPM)
    target_motor_rpm            目标电机转速 (RPM)，与 target_ratio 二选一
    target_ratio                目标转速占空载比例，如 0.6、0.7
    gear_ratio                  总减速比 (电机 : 轮)
    target_linear_speed_mps     目标线速度 (m/s)，与 reference_* 二选一
    reference_wheel_diameter_mm 已知轮径 (mm)
    reference_wheel_rpm         已知轮端转速 (RPM)
    """

    if target_motor_rpm is None:
        if target_ratio is None:
            raise ValueError("必须提供 target_motor_rpm 或 target_ratio")
        target_motor_rpm = motor_free_rpm * target_ratio

    target_wheel_rpm = target_motor_rpm / gear_ratio

    # 情况 A：已知目标线速度
    if target_linear_speed_mps is not None:
        wheel_diameter_m = (
            target_linear_speed_mps * 60
            / (math.pi * target_wheel_rpm)
        )
        return {
            "target_motor_rpm": target_motor_rpm,
            "target_wheel_rpm": target_wheel_rpm,
            "wheel_diameter_mm": wheel_diameter_m * 1000,
        }

    # 情况 B：用参考点比例缩放
    if reference_wheel_diameter_mm is not None and reference_wheel_rpm is not None:
        wheel_diameter_mm = (
            reference_wheel_diameter_mm
            * reference_wheel_rpm
            / target_wheel_rpm
        )
        return {
            "target_motor_rpm": target_motor_rpm,
            "target_wheel_rpm": target_wheel_rpm,
            "wheel_diameter_mm": wheel_diameter_mm,
        }

    raise ValueError("必须提供目标线速度或参考轮径 + 轮端转速")
