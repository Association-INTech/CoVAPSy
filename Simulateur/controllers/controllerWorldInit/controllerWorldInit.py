from typing import Tuple
import numpy as np
from controller import Supervisor

import os
import sys

# this is necessary because we are not in a package context
# so we cannot do from ..config import *
# SO EVEN IF IT LOOKS UGLY, WE HAVE TO DO THIS
script_dir = os.path.dirname(os.path.abspath(__file__))
controllers_path = os.path.join(script_dir, '../..')
sys.path.append(controllers_path)

from config import *



def create_nodes(supervisor: Supervisor, n_vehicles: int):
    """
    Creates n_vehicles vehicles in the simulation
    for each vehicle, create an emitter and a receiver in the supervisor
    """

    root = supervisor.getRoot()
    root_children_field = root.getField("children")

    proto_string = f"""
    DEF WorldSupervisor Robot {{
        supervisor TRUE
        name "WorldSupervisor"
        controller "controllerWorldSupervisor"
        children [
            {"\n".join([f' Emitter  {{name "supervisor_emitter_{i}"}}' for i in range(n_vehicles)])}
            {"\n".join([f' Receiver {{name "supervisor_receiver_{i}"}}' for i in range(n_vehicles)])}
        ]
    }}
    """
    root_children_field.importMFNodeFromString(-1, proto_string)


    for i in range(n_vehicles):
        proto_string = f"""
        DEF TT02_{i} TT02_2023b {{
            name "TT02_{i}"
            controller "controllerVehicleDriver"
            color {" ".join((np.random.rand(3) * 0.4).astype(str))}
            lidar_horizontal_resolution {lidar_horizontal_resolution}
            camera_horizontal_resolution {camera_horizontal_resolution}
        }}
        """
        root_children_field.importMFNodeFromString(-1, proto_string)


    for i in range(n_stupid_vehicles):
        proto_string = f"""
        DEF TT02_{i} TT02_2023b {{
            name "TT02_{i}"
            controller "controller_violet"
            color {" ".join((np.random.rand(3) * 0.8).astype(str))}
            lidar_horizontal_resolution {lidar_horizontal_resolution}
            camera_horizontal_resolution {camera_horizontal_resolution}
        }}
        """
        root_children_field.importMFNodeFromString(-1, proto_string)


if __name__ == "__main__":
    S = Supervisor()
    create_nodes(S, n_vehicles)
