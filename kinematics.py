import math
from typing import List, Tuple
from geometry2d import Transform2d, Twist2d, Twist2dVelocity, Vector2d
import numpy as np
from dataclasses import dataclass

wheel_base = 139.00000 / 1000.0  # 139 mm
track_width = 139.00000 / 1000.0  # 139 mm


fl_pos = Vector2d(wheel_base / 2, track_width / 2)  # v1
fr_pos = Vector2d(wheel_base / 2, -track_width / 2)  # v2
bl_pos = Vector2d(-wheel_base / 2, track_width / 2)  # v3
br_pos = Vector2d(-wheel_base / 2, -track_width / 2)  # v4


@dataclass
class WheelSpeeds:
    front_left: float
    front_right: float
    back_left: float
    back_right: float

    def from_id(self, id: int) -> float:
        assert id % 2 == 1
        if id == 1:
            return self.front_left
        elif id == 3:
            return self.front_right
        elif id == 5:
            return self.back_left
        elif id == 7:
            return self.back_right
        else:
            raise ValueError(f"Invalid id: {id}")


@dataclass
class ModuleAngles:
    front_left_angle: float
    front_right_angle: float
    back_left_angle: float
    back_right_angle: float

    def from_id(self, id: int) -> float:
        assert id % 2 == 0
        if id == 2:
            return self.front_left_angle
        elif id == 4:
            return self.front_right_angle
        elif id == 6:
            return self.back_left_angle
        elif id == 8:
            return self.back_right_angle
        else:
            raise ValueError(f"Invalid id: {id}")

    def to_list(self) -> List[float]:
        return [self.front_left_angle, self.front_right_angle, self.back_left_angle, self.back_right_angle]

    def to_list_degrees(self) -> List[float]:
        return [math.degrees(angle) for angle in self.to_list()]


def twist_to_wheel_speeds(twist: Twist2dVelocity, dt: float) -> Tuple[WheelSpeeds, ModuleAngles]:

    transform = Transform2d(twist.vx * dt, twist.vy * dt, twist.w * dt)
    twist = transform.log()
    twist = Twist2dVelocity(twist.dx / dt, twist.dy / dt, twist.dyaw / dt)

    twist = np.array([twist.vx, twist.vy, twist.w])

    if epsilon_equals(twist[0], 0, 0.05) and epsilon_equals(twist[1], 0, 0.05) and not epsilon_equals(twist[2], 0, 0.05):
        theta1 = np.deg2rad(-45)
        theta2 = np.deg2rad(45)
        theta3 = np.deg2rad(-45)
        theta4 = np.deg2rad(45)

        radius = np.sqrt(wheel_base * wheel_base)  # hypotenuse divided by 2 to get distance from center to wheel
        # w = v / r
        # v = w * r
        v1 = -twist[2] * radius
        v2 = twist[2] * radius
        v3 = -twist[2] * radius
        v4 = twist[2] * radius
    else:
        transition = np.array(
            [
                [1, 0, -fl_pos.y],
                [0, 1, fl_pos.x],
                [1, 0, -fr_pos.y],
                [0, 1, fr_pos.x],
                [1, 0, -bl_pos.y],
                [0, 1, bl_pos.x],
                [1, 0, -br_pos.y],
                [0, 1, br_pos.x],
            ]
        )
        speeds = np.dot(transition, twist.transpose())

        v1 = np.sqrt(speeds[0] ** 2 + speeds[1] ** 2)
        v2 = np.sqrt(speeds[2] ** 2 + speeds[3] ** 2)
        v3 = np.sqrt(speeds[4] ** 2 + speeds[5] ** 2)
        v4 = np.sqrt(speeds[6] ** 2 + speeds[7] ** 2)

        theta1 = np.arctan2(speeds[1], speeds[0])
        theta2 = np.arctan2(speeds[3], speeds[2])
        theta3 = np.arctan2(speeds[5], speeds[4])
        theta4 = np.arctan2(speeds[7], speeds[6])

    return WheelSpeeds(front_left=v1, front_right=v2, back_left=v3, back_right=v4), ModuleAngles(
        front_left_angle=theta1, front_right_angle=theta2, back_left_angle=theta3, back_right_angle=theta4
    )


if __name__ == "__main__":
    twist = Twist2dVelocity(1.0, 0.0, 0.0)
    speeds, angles = twist_to_wheel_speeds(twist, 0.05)

    print(angles.to_list_degrees())
    print(speeds.front_left)
    print(speeds.front_right)
    print(speeds.back_left)
    print(speeds.back_right)


def epsilon_equals(a: float, b: float, epsilon: float) -> bool:
    return abs(a - b) < epsilon
