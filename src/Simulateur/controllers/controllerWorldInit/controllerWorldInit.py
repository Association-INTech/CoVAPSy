from typing import Tuple
import numpy as np
from controller import Supervisor


def create_nodes(supervisor: Supervisor, n_envs: int, lidar_horizontal_resolution: int, lidar_max_range: float):
    """
    Creates n_envs vehicles in the simulation
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
            {"\n".join([f' Emitter  {{name "supervisor_emitter_{i}"}}' for i in range(n_envs)])}
            {"\n".join([f' Receiver {{name "supervisor_receiver_{i}"}}' for i in range(n_envs)])}
        ]
    }}
    """

    print(proto_string)
    root_children_field.importMFNodeFromString(-1, proto_string)

    for i in range(n_envs):
        proto_string = f"""
        DEF TT02_{i} TT02_2023b {{
            name "TT02_{i}"
            controller "controllerVehicleDriver"
            color 0.5 0 0.6
            lidarHorizontalResolution {lidar_horizontal_resolution}
            lidarMaxRange {lidar_max_range}
        }}
        """
        root_children_field.importMFNodeFromString(-1, proto_string)


if __name__ == "__main__":
    S = Supervisor()
    n_envs = 2
    lidar_horizontal_resolution = 512
    lidar_max_range = 12.0

    create_nodes(S, n_envs, lidar_horizontal_resolution, lidar_max_range)
