#!/usr/bin/python3 -B

import asyncio
import math
import moteus
import moteus_pi3hat
import time
from tqdm import tqdm

AZIMUTH_RATIO = 12.0 / 83.0

def angle_wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

def calculate_swerve_angle(position: float) -> float:
    return angle_wrap(position * 2 * math.pi * AZIMUTH_RATIO)

def calculate_target_position_delta(reference_azimuth_angle, estimated_angle):
    angle_difference = angle_wrap(reference_azimuth_angle - estimated_angle)
    return angle_difference / AZIMUTH_RATIO

async def main():
    transport = moteus_pi3hat.Pi3HatRouter(
        servo_bus_map = {
            1: [1, 2, 3],
            2: [4, 5, 6],
            3: [7, 8]
        },
    )

    azimuth_ids = [2, 4, 6, 8]
    drive_ids = [1, 3, 5, 7]

    servos = {
        servo_id: moteus.Controller(id=servo_id, transport=transport)
        for servo_id in azimuth_ids + drive_ids
    }

    # Stop all servos
    await transport.cycle([x.make_stop() for x in servos.values()])

    commands = []
    for id in azimuth_ids:
        commands.append(servos[id].make_rezero())
    print("Resetting encoders on azimuth modules...")
    await transport.cycle(commands)
    print("Successfully reset encoders on azimuth modules...")

    reference_angle = math.pi / 2  # 90 degrees
    gain = 0.1

    try:
        while True:
            commands = []
            for id in azimuth_ids:
                commands.append(servos[id].make_position(position=math.nan, velocity=0.0, query=True))

            results = await transport.cycle(commands)

            measured_module_positions = {
                result.id: result.values[moteus.Register.POSITION]
                for result in results if result.id in azimuth_ids
            }

            commands = []
            for id in azimuth_ids:
                pos = measured_module_positions[id]
                current_angle = calculate_swerve_angle(pos)
                delta = calculate_target_position_delta(reference_angle, current_angle)
                reference = pos + gain * delta

                print(f"ID: {id}, Current Angle: {math.degrees(current_angle):.2f}°", end = " ")

                commands.append(servos[id].make_position(
                    position=math.nan,
                    velocity=0.0,
                    maximum_torque=0.0,
                    query=True
                ))
            print("")

            await transport.cycle(commands)
            await asyncio.sleep(0.02)

    except KeyboardInterrupt:
        print("\nStopping all servos...")
        await transport.cycle([x.make_stop() for x in servos.values()])

if __name__ == '__main__':
    asyncio.run(main())