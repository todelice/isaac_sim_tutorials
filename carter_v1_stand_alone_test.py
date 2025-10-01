# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import argparse
import sys

import carb
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.storage.native import get_assets_root_path
from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
from isaacsim.robot.wheeled_robots.robots import WheeledRobot


parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()


assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()

asset_path = assets_root_path + "/Isaac/Robots/NVIDIA/Carter/carter_v1.usd"
my_carter = my_world.scene.add(
    WheeledRobot(
        prim_path="/World/Carter",
        name="my_carter",
        wheel_dof_names=["left_wheel", "right_wheel"],
        create_robot=True,
        usd_path=asset_path,
        position=np.array([0, 0.0, 0.05]),
    )
)

cube = my_world.scene.add(
    DynamicCuboid(
        name="cube",
        position=np.array([0.3, 0.3, 0.3]),
        prim_path="/World/Cube",
        scale=np.array([0.0515, 0.0515, 0.0515]),
        size=1.0,
        color=np.array([0, 0, 1]),
    )
)
my_world.scene.add_default_ground_plane()
my_controller = DifferentialController(name="simple_control", wheel_radius=0.03, wheel_base=0.1125)
my_world.reset()


reset_needed = False
task_completed = False
i = 0
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
        task_completed = False
    if my_world.is_playing():
        if i >= 0 and i < 1000:
            my_carter.apply_wheel_actions(my_controller.forward(command=[0.1, 0]))
            print(my_carter.get_linear_velocity())
        pass
        if i>2000:
            break
        i += 1

    if args.test is True:
        break


simulation_app.close()