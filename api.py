# /// script
# requires-python = ">=3.11"
# dependencies = [
#   "fastapi",
#   "uvicorn",
#   "pydantic",
# ]
# ///

import math
from fastapi import FastAPI
from fastapi.routing import APIRoute
from pydantic import BaseModel, Field
import uvicorn
from contextlib import asynccontextmanager
import asyncio
from multiprocessing import Process, JoinableQueue
import main

command_queue = JoinableQueue()


@asynccontextmanager
async def lifespan(app: FastAPI):
    # Run at startup
    asyncio.create_task(main.main())
    # def start_main(command_queue: JoinableQueue):
    #     asyncio.run(main.main(command_queue))

    # p = Process(target=start_main, args=(command_queue,))
    # p.start()
    yield
    # Run on shutdown (if required)
    print("Shutting down...")
    # p.terminate()


def custom_generate_unique_id(route: APIRoute) -> str:
    return route.name


app = FastAPI(generate_unique_id_function=custom_generate_unique_id, lifespan=lifespan)


class FieldRelativeVelocity(BaseModel):
    """Field relative velocity"""

    vx: float = Field(..., description="Velocity in the x direction. Units: m/s")
    vy: float = Field(..., description="Velocity in the y direction. Units: m/s")
    omega: float = Field(..., description="Angular velocity. Units: rad/s")


class RobotCommandResponse(BaseModel):
    """Response to a robot command"""

    success: bool = Field(..., description="Whether the command was successful or not")


@app.post("/set-velocity", tags=["robot_control"])
async def set_velocity(new_velocity: FieldRelativeVelocity) -> RobotCommandResponse:
    """Set the field relative velocity of the robot. The units are m/s and rad/s."""

    main.reference_vx = new_velocity.vx
    main.reference_vy = new_velocity.vy
    main.reference_w = math.radians(new_velocity.omega)
    # command_queue.put(new_velocity)

    return RobotCommandResponse(success=True)


uvicorn.run(app, host="0.0.0.0", port=8000)
