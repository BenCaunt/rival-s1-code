# /// script
# requires-python = ">=3.11"
# dependencies = [
#   "fastapi",
#   "uvicorn",
#   "pydantic",
# ]
# ///

from fastapi import FastAPI
from fastapi.routing import APIRoute
from pydantic import BaseModel, Field
import uvicorn
from contextlib import asynccontextmanager
import asyncio
from main import main


@asynccontextmanager
async def lifespan(app: FastAPI):
    # Run at startup
    asyncio.create_task(main())
    yield
    # Run on shutdown (if required)
    print("Shutting down...")


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


velocity = FieldRelativeVelocity(vx=0, vy=0, omega=0)


@app.post("/set-velocity", tags=["robot_control"])
async def set_velocity(new_velocity: FieldRelativeVelocity) -> RobotCommandResponse:
    """Set the field relative velocity of the robot. The units are m/s and rad/s."""

    global velocity
    velocity = new_velocity

    return RobotCommandResponse(success=True)


uvicorn.run(app, host="0.0.0.0", port=8000)
