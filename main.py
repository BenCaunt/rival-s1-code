#!/usr/bin/python3 -B

import asyncio
import math
import moteus
import moteus_pi3hat
import time
from tqdm import tqdm

AZIMUTH_RATIO = 12.0 / 83.0
DRIVE_REDUCTION = 17.0 / 54.0

DRIVE_DIAMETER = 0.075 # 75 mm
DRIVE_CIRCUMFERENCE = DRIVE_DIAMETER * math.pi


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
    results = await transport.cycle([x.make_stop(query=True) for x in servos.values()])

    initial_module_positions = {
        result.id: result.values[moteus.Register.POSITION]
        for result in results if result.id in azimuth_ids
    }
    print(initial_module_positions)

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

                current_angle = calculate_swerve_angle(measured_module_positions[id]) - calculate_swerve_angle(initial_module_positions[id])
                current_angle = angle_wrap(current_angle)
                target_angle = reference_angle
                error = angle_wrap(target_angle - current_angle)
                target_position_delta = calculate_target_position_delta(reference_angle, current_angle)

                commands.append(servos[id].make_position(
                    position=measured_module_positions[id] + target_position_delta,
                    velocity=0.0,
                    maximum_torque=1.5,
                    velocity_limit=90.0,
                    accel_limit=120.0,
                    query=True
                ))


                # if id == 8:
                #     print(f"ID: {id}, Current Angle Error: {math.degrees(error):.2f}°", end = " ")
                #     print(f"Target Position Delta: {target_position_delta:.2f}", end = " ")
                #     print(f"Current Angle: {math.degrees(current_angle):.2f}°", end = " ")
                #     print(f"raw position: {measured_module_positions[id]:.2f}", end = " ")

            reference_velocity = 0.25 # m/s 
            reference_wheel_speed = wheel_speed_to_motor_speed(reference_velocity)
            for id in drive_ids:
                commands.append(servos[id].make_position(
                    position=math.nan,
                    velocity=reference_wheel_speed,
                    maximum_torque=1.5,
                    query=True)
                )
            
            print("")

            results = await transport.cycle(commands)

            measured_module_positions = {
                result.id: result.values[moteus.Register.POSITION]
                for result in results if result.id in azimuth_ids
            }

            # print motor speed vs expected speed
            for id in drive_ids:
                motor_speed = results[id].values[moteus.Register.VELOCITY]
                wheel_speed = motor_speed_to_wheel_speed(motor_speed)
                print(f"ID: {id}, Expected Wheel Speed: {reference_velocity:.2f} m/s, Measured Wheel Speed: {wheel_speed:.2f} m/s")
                # expected motor vs actual motor speed 
                print(f"ID: {id}, Expected Motor Speed: {reference_wheel_speed:.2f}, Measured Motor Speed: {motor_speed:.2f}")


            await asyncio.sleep(0.005)

    except KeyboardInterrupt:
        print("\nStopping all servos...")
        await transport.cycle([x.make_stop() for x in servos.values()])

if __name__ == '__main__':
    asyncio.run(main())