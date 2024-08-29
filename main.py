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
            1:[1,2,3],
            2:[4,5,6],
            3:[7,8]
        },
    )

    azimuth_ids = [2,4,6,8]
    drive_ids = [1,3,5,7]

    servos = {
        servo_id : moteus.Controller(id=servo_id, transport=transport)
        for servo_id in [1, 2, 3, 4, 5, 6, 7, 8]
    }

    await transport.cycle([x.make_stop() for x in servos.values()])

    position_offsets = {}
    for id in tqdm(azimuth_ids):
        position_offsets[id] = 0.0
        for _ in range(10):
            try:
                results = await transport.cycle([servos[id].make_position(position=math.nan, velocity=0.0, query=True)])
                for result in results:
                    if isinstance(result, moteus.Result) and result.id == id:
                        position_offsets[id] += result.values[moteus.Register.POSITION]
                        break
                else:
                    print(f"No valid result found for servo {id}")
            except Exception as e:
                print(f"Error querying servo {id}: {e}")
        position_offsets[id] /= 10

    while True:
        now = time.time()

        gain = 0.1 
        reference_angle = math.pi/2 # 90 degrees

        measured_module_positions = {}
        commands = []

        for id in azimuth_ids:
            try:
                results = await transport.cycle([servos[id].make_position(position=math.nan, velocity=0.0, query=True)])
                for result in results:
                    if isinstance(result, moteus.Result) and result.id == id:
                        measured_position = result.values[moteus.Register.POSITION] - position_offsets[id]
                        measured_module_positions[id] = measured_position
                        print(f"Measured position for {id}: {measured_position}, {calculate_swerve_angle(measured_position)}")
                        break
                else:
                    print(f"No valid result found for servo {id}")
            except Exception as e:
                print(f"Error querying servo {id}: {e}")

        for id in azimuth_ids:
            try:
                pos = measured_module_positions[id]
                delta = calculate_target_position_delta(reference_angle, calculate_swerve_angle(pos))
                reference = pos + delta
                print(f"Reference position for {id}: {reference}")
                print(f"Delta for {id}: {delta}")

                commands.append(servos[id].make_position(
                    position=reference,
                    velocity=0.0,
                    query=True))
            except KeyError:
                print(f"Could not find position for {id}")

        try:
            results = await transport.cycle(commands)
            for result in results:
                if isinstance(result, moteus.Result):
                    print(f"Servo {result.id}: Position {result.values[moteus.Register.POSITION]}, Velocity {result.values[moteus.Register.VELOCITY]}")
        except Exception as e:
            print(f"Error during command cycle: {e}")

        await asyncio.sleep(0.04)

if __name__ == '__main__':
    asyncio.run(main())
