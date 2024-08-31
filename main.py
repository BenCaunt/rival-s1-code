#!/usr/bin/python3 -B

import asyncio
import math
import moteus
import moteus_pi3hat
import time
from tqdm import tqdm

AZIMUTH_RATIO = 12.0 / 83.0

def angle_wrap(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def calculate_swerve_angle(position: float) -> float:
    return angle_wrap(position * 2 * math.pi * AZIMUTH_RATIO)

def calculate_target_position_delta(reference_azimuth_angle, estimated_angle):
    angle_difference = angle_wrap(reference_azimuth_angle - estimated_angle)
    return angle_difference / (2 * math.pi * AZIMUTH_RATIO)

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
        commands.append(servos[id].make_rezero(query=True))
    print("Resetting encoders on azimuth modules...")
    results = await transport.cycle(commands)
    print("Successfully reset encoders on azimuth modules...")
    initial_module_positions = {
        result.id: result.values[moteus.Register.POSITION]
        for result in results if result.id in azimuth_ids
    }

    reference_angle = math.pi / 2  # 90 degrees
    gain = 0.1

    measured_module_positions = {
        2: 0.0,
        4: 0.0,
        6: 0.0,
        8: 0.0
    }

    try:
        while True:
        
            commands = []
            for id in azimuth_ids:

                current_angle = calculate_swerve_angle(measured_module_positions[id] - initial_module_positions[id])
                target_angle = reference_angle
                error = angle_wrap(target_angle - current_angle)
                target_position_delta = calculate_target_position_delta(reference_angle, current_angle)

                commands.append(servos[id].make_position(
                    position=measured_module_positions[id] + target_position_delta,
                    velocity=0.0,
                    maximum_torque=0.5,
                    query=True
                ))


                print(f"ID: {id}, Current Angle Error: {math.degrees(error):.2f}°", end = " ")

                
            print("")

            results = await transport.cycle(commands)

            measured_module_positions = {
                result.id: result.values[moteus.Register.POSITION]
                for result in results if result.id in azimuth_ids
            }
            await asyncio.sleep(0.005)

    except KeyboardInterrupt:
        print("\nStopping all servos...")
        await transport.cycle([x.make_stop() for x in servos.values()])

if __name__ == '__main__':
    asyncio.run(main())