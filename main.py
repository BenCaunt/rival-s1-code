#!/usr/bin/python3 -B

import asyncio
import math

import numpy as np
import moteus
import moteus_pi3hat
import time
from tqdm import tqdm

AZIMUTH_RATIO = 12.0 / 83.0
DRIVE_REDUCTION = 17.0 / 54.0

DRIVE_DIAMETER = 0.075  # 75 mm
DRIVE_CIRCUMFERENCE = DRIVE_DIAMETER * math.pi

# GLOBAL STATE
reference_angle = math.radians(90.0)  # radians
reference_velocity = 0.0  # m/s
gain = 0.1

from kinematics import twist_to_wheel_speeds, WheelSpeeds, ModuleAngles
from geometry2d import Twist2dVelocity


def angle_wrap(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def calculate_swerve_angle(position: float) -> float:
    return angle_wrap(position * 2 * math.pi * AZIMUTH_RATIO)


def wheel_speed_to_motor_speed(wheel_speed: float) -> float:
    return wheel_speed / (DRIVE_CIRCUMFERENCE * DRIVE_REDUCTION)


def motor_speed_to_wheel_speed(motor_speed: float) -> float:
    return motor_speed * (DRIVE_CIRCUMFERENCE * DRIVE_REDUCTION)


def calculate_target_position_delta(reference_azimuth_angle, estimated_angle):
    angle_difference = angle_wrap(reference_azimuth_angle - estimated_angle)
    return angle_difference / (2 * math.pi * AZIMUTH_RATIO)


async def main():
    transport = moteus_pi3hat.Pi3HatRouter(
        servo_bus_map={1: [1, 2, 3], 2: [4, 5, 6], 3: [7, 8]},
    )

    azimuth_ids = [2, 4, 6, 8]
    drive_ids = [1, 3, 5, 7]

    servos = {servo_id: moteus.Controller(id=servo_id, transport=transport) for servo_id in azimuth_ids + drive_ids}

    drive_directions = {1: 1, 3: 1, 5: -1, 7: -1}

    # Stop all servos
    results = await transport.cycle([x.make_stop(query=True) for x in servos.values()])

    initial_module_positions = {
        result.id: result.values[moteus.Register.POSITION] for result in results if result.id in azimuth_ids
    }

    measured_module_positions = {2: 0.0, 4: 0.0, 6: 0.0, 8: 0.0}

    try:
        while True:

            reference = Twist2dVelocity(1.0, 0.0, 0.0)
            wheel_speeds, module_angles = twist_to_wheel_speeds(reference)

            commands = []
            for id in azimuth_ids:

                current_angle = calculate_swerve_angle(measured_module_positions[id]) - calculate_swerve_angle(
                    initial_module_positions[id]
                )
                current_angle = angle_wrap(current_angle)
                target_angle = -(np.pi / 2.0)  # module_angles.from_id(id)
                target_angle = angle_wrap(target_angle)
                error = angle_wrap(target_angle - current_angle)
                target_position_delta = calculate_target_position_delta(target_angle, current_angle)

                commands.append(
                    servos[id].make_position(
                        position=measured_module_positions[id] + target_position_delta,
                        velocity=0.0,
                        maximum_torque=1.5,
                        velocity_limit=90.0,
                        accel_limit=120.0,
                        query=True,
                    )
                )

            for id in drive_ids:
                commands.append(
                    servos[id].make_position(
                        position=math.nan,
                        velocity=wheel_speed_to_motor_speed(wheel_speeds.from_id(id)) * drive_directions[id],
                        maximum_torque=1.5,
                        query=True,
                    )
                )

            results = await transport.cycle(commands)

            measured_module_positions = {
                result.id: result.values[moteus.Register.POSITION] for result in results if result.id in azimuth_ids
            }

            await asyncio.sleep(0.005)

    except KeyboardInterrupt:
        print("\nStopping all servos...")
        await transport.cycle([x.make_stop() for x in servos.values()])


if __name__ == "__main__":
    asyncio.run(main())
