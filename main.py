#!/usr/bin/python3 -B

# Copyright 2023 mjbots Robotic Systems, LLC.  info@mjbots.com
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
This example commands multiple servos connected to a pi3hat.  It
uses the .cycle() method in order to optimally use the pi3hat
bandwidth.
"""

import asyncio
import math
import moteus
import moteus_pi3hat
import time
from tqdm import tqdm


def angle_wrap(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

AZIMUTH_RATIO = 12.0 / 83.0

def calculate_swerve_angle(position: float) -> float:
    """
    Given the module position in rotations, calculate the angle of the swerve module.

    The swerve module has a 12:83 gear ratio, so the module position is 12/83 of the azimuth
    position.

    Args:
        position: The module position in rotations.
    """

    return angle_wrap(position * 2 * math.pi * AZIMUTH_RATIO)

def calculate_target_position_delta(reference_azimuth_angle, estimated_angle):
    """
    Given the reference azimuth angle and the estimated angle of the swerve module, calculate the
    target position for the module.

    Args:
        reference_azimuth_angle: The azimuth angle of the robot.
        estimated_angle: The estimated angle of the swerve module.
    """

    # Calculate the angle difference between the reference azimuth angle and the estimated angle.
    angle_difference = angle_wrap(reference_azimuth_angle - estimated_angle)

    # Calculate the target position for the swerve module.
    return angle_difference / AZIMUTH_RATIO


async def main():
    # We will be assuming a system where there are 4 servos, each
    # attached to a separate pi3hat bus.  The servo_bus_map argument
    # describes which IDs are found on which bus.
    transport = moteus_pi3hat.Pi3HatRouter(
        servo_bus_map = {
            1:[1,2,3],
            2:[4,5,6],
            3:[7,8]
        },
    )

    azimuth_ids = [2,4,6,8]
    drive_ids = [1,3,5,7]



    # We create one 'moteus.Controller' instance for each servo.  It
    # is not strictly required to pass a 'transport' since we do not
    # intend to use any 'set_*' methods, but it doesn't hurt.
    #
    # This syntax is a python "dictionary comprehension":
    # https://docs.python.org/3/tutorial/datastructures.html#dictionaries
    servos = {
        servo_id : moteus.Controller(id=servo_id, transport=transport)
        for servo_id in [1, 2, 3, 4, 5, 6,7,8]
    }

    # We will start by sending a 'stop' to all servos, in the event
    # that any had a fault.
    await transport.cycle([x.make_stop() for x in servos.values()])

    position_offsets = {}
    for id in tqdm(azimuth_ids):
        position_offsets[id] = 0.0
        for _ in range(10):
            results = await transport.cycle([servos[id].make_position(position=math.nan, velocity=0.0, query=True)])
            for result in results:
                if result.id == id:
                    assert result.values[moteus.Register.POSITION] is float
                    position_offsets[id] += result.values[moteus.Register.POSITION]
                    break
        position_offsets[id] /= 10

    while True:
        # The 'cycle' method accepts a list of commands, each of which
        # is created by calling one of the `make_foo` methods on
        # Controller.  The most common thing will be the
        # `make_position` method.

        now = time.time()

        # For now, we will just construct a position command for each
        # of the 4 servos, each of which consists of a sinusoidal
        # velocity command starting from wherever the servo was at to
        # begin with.
        #
        # 'make_position' accepts optional keyword arguments that
        # correspond to each of the available position mode registers
        # in the moteus reference manual.

        gain = 0.1 
        # commands = [
        #     servos[1].make_position(
        #         position=math.nan,
        #         velocity=gain*math.sin(now),
        #         query=True),
        #     servos[2].make_position(
        #         position=math.nan,
        #         velocity=gain*math.sin(now + 1),
        #         query=True),
        #     servos[3].make_position(
        #         position=math.nan,
        #         velocity=gain*math.sin(now),
        #         query=True),
        #     servos[4].make_position(
        #         position=math.nan,
        #         velocity=gain*math.sin(now),
        #         query=True),
        #     servos[5].make_position(
        #         position=math.nan,
        #         velocity=gain*math.sin(now + 1),
        #         query=True),
        #     servos[6].make_position(
        #         position=math.nan,
        #         velocity=gain*math.sin(now),
        #         query=True),       
        #     servos[7].make_position(
        #         position=math.nan,
        #         velocity=gain*math.sin(now),
        #         query=True),
        #     servos[8].make_position(
        #         position=math.nan,
        #         velocity=gain*math.sin(now + 1),
        #         query=True),         
        # ]

        reference_angle = math.pi/2 # 90 degrees



        measured_module_positions = {}
        for id in azimuth_ids:
            for result in results:
                if result.id == id:
                    measured_module_positions[id] = result.values[moteus.Register.POSITION] - position_offsets[id]
                    print(f"Measured position for {id}: {measured_module_positions[id]}, {calculate_swerve_angle(measured_module_positions[id])}")
                    break
        assert len(measured_module_positions) == len(azimuth_ids)

    
        for id in azimuth_ids:
            pos = measured_module_positions[id]

            commands = [
                servos[id].make_position(
                    position=pos + calculate_target_position_delta(reference_angle, calculate_swerve_angle(pos)),
                    velocity=0.0,
                    query=True)
            ]

        assert len(commands) == len(azimuth_ids)

        # By sending all commands to the transport in one go, the
        # pi3hat can send out commands and retrieve responses
        # simultaneously from all ports.  It can also pipeline
        # commands and responses for multiple servos on the same bus.
        results = await transport.cycle(commands)

        # The result is a list of 'moteus.Result' types, each of which
        # identifies the servo it came from, and has a 'values' field
        # that allows access to individual register results.
        #
        # Note: It is possible to not receive responses from all
        # servos for which a query was requested.
        #
        # Here, we'll just print the ID, position, and velocity of
        # each servo for which a reply was returned.
        # print(", ".join(
        #     f"({result.arbitration_id} " +
        #     f"{result.values[moteus.Register.POSITION]} " +
        #     f"{result.values[moteus.Register.VELOCITY]})"
        #     for result in results))

        for i,result in enumerate(results):
            print("______________")
            print(f"{result.id}")
            print("______________")
            

        # We will wait 20ms between cycles.  By default, each servo
        # has a watchdog timeout, where if no CAN command is received
        # for 100ms the controller will enter a latched fault state.
        await asyncio.sleep(0.02)



if __name__ == '__main__':
    asyncio.run(main())
