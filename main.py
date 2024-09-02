#!/usr/bin/python3 -B

import asyncio
import math

import numpy as np
import moteus
import moteus_pi3hat
import time
from tqdm import tqdm
from multiprocessing import JoinableQueue

AZIMUTH_RATIO = 12.0 / 83.0
DRIVE_REDUCTION = 17.0 / 54.0

DRIVE_DIAMETER = 0.075  # 75 mm
DRIVE_CIRCUMFERENCE = DRIVE_DIAMETER * math.pi

# GLOBAL STATE
reference_vx = 0.0  # m/s
reference_vy = 0.0  # m/s
reference_w = 0.0  # rad/s

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
    module_scaling = {2: 1.0, 4: 1.0, 6: 1.0, 8: 1.0}
    module_inversions = {2: False, 4: False, 6: False, 8: False}

    try:
        loop_start = time.monotonic()
        dt = 0.005

        while True:
            dt = time.monotonic() - loop_start
            loop_start = time.monotonic()

            # if not command_queue.empty():
            #     command = command_queue.get()
            #     global reference_vx, reference_vy, reference_w

            #     reference_vx = command.vx
            #     reference_vy = command.vy
            #     reference_w = command.omega

            #     command_queue.task_done()

            #     print(f"reference_vx: {reference_vx}, reference_vy: {reference_vy}, reference_w: {reference_w}")

            reference = Twist2dVelocity(reference_vx, reference_vy, reference_w)
            wheel_speeds, module_angles = twist_to_wheel_speeds(reference, dt)
            # print(module_angles.to_list_degrees())

            commands = []
            for id in azimuth_ids:

                current_angle = calculate_swerve_angle(measured_module_positions[id]) - calculate_swerve_angle(
                    initial_module_positions[id]
                )
                current_angle = angle_wrap(current_angle)
                target_angle = module_angles.from_id(id)
                # this makes it so + is ccw with module rotation.
                target_angle = -angle_wrap(target_angle)

                # if module_inversions[id]:
                #     target_angle = angle_wrap(np.pi - target_angle)

                error = angle_wrap(target_angle - current_angle)
                print(f"error: {math.degrees(error)}")

                module_scaling[id] = np.cos(np.clip(error, -np.pi / 2, np.pi / 2))

                # if id == 2 and module_inversions[id]:
                #     print(f"inverting module {id}")

                if abs(error) > np.pi / 2:
                    module_inversions[id] = not module_inversions[id]

                target_position_delta = calculate_target_position_delta(target_angle, current_angle)

                commands.append(
                    servos[id].make_position(
                        position=measured_module_positions[id] + target_position_delta,
                        velocity=0.0,
                        maximum_torque=1.7,
                        velocity_limit=90.0,
                        accel_limit=120.0,
                        query=True,
                    )
                )

            for id in drive_ids:
                sign = 1.0

                # if module_inversions[id + 1]:
                #     sign = -1.0

                commands.append(
                    servos[id].make_position(
                        position=math.nan,
                        velocity=module_scaling[id + 1]
                        * sign
                        * wheel_speed_to_motor_speed(wheel_speeds.from_id(id))
                        * drive_directions[id],
                        maximum_torque=1.5,
                        accel_limit=60.0,
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
