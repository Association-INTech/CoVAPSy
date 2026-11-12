import random

from controller import Supervisor

from simulation import config as c


car_colors = [
    [0.0, 0.0, 1.0],  # Blue
    [1.0, 1.0, 0.0],  # Yellow
    [0.0, 1.0, 1.0],  # Cyan
    [1.0, 0.0, 1.0],  # Magenta
    [1.0, 0.5, 0.0],  # Orange
    [0.5, 0.0, 1.0],  # Violet
    [1.0, 1.0, 1.0],  # White
    [0.0, 0.5, 1.0],  # Azure
    [1.0, 0.0, 0.5],  # Rose
    [0.0, 0.5, 0.0],  # Green
]


def create_nodes():
    """
    Creates n_vehicles vehicles in the simulation
    for each vehicle, create an emitter and a receiver in the supervisor
    """

    supervisor = Supervisor()

    root = supervisor.getRoot()
    root_children_field = root.getField("children")

    proto_string = f"""
    DEF WorldSupervisor Robot {{
        supervisor TRUE
        name "WorldSupervisor"
        controller "controller_world_supervisor"
        children [
            {"\n".join([f' Emitter  {{name "supervisor_emitter_{i}"}}' for i in range(c.n_vehicles)])}
            {"\n".join([f' Receiver {{name "supervisor_receiver_{i}"}}' for i in range(c.n_vehicles)])}
        ]
    }}
    """
    root_children_field.importMFNodeFromString(-1, proto_string)

    for i in range(c.n_vehicles):
        color = random.choice(car_colors)
        proto_string = f"""
        DEF TT02_{i} TT02_2023b {{
            name "TT02_{i}"
            controller "controller_vehicle_driver"
            color {color[0]} {color[1]} {color[2]}
            lidar_horizontal_resolution {c.lidar_horizontal_resolution}
            camera_horizontal_resolution {c.camera_horizontal_resolution or 1}
            camera_vertical_resolution {c.camera_vertical_resolution or 1}
        }}
        """
        root_children_field.importMFNodeFromString(-1, proto_string)

    for i in range(c.n_vehicles, c.n_vehicles + c.n_stupid_vehicles):
        proto_string = f"""
        DEF TT02_{i} TT02_2023b {{
            name "TT02_{i}"
            controller "controller_violet"
            color 0.0 0.0 0.0
            lidar_horizontal_resolution {c.lidar_horizontal_resolution}
            camera_horizontal_resolution {c.camera_horizontal_resolution or 1}
            camera_vertical_resolution {c.camera_vertical_resolution or 1}
        }}
        """
        root_children_field.importMFNodeFromString(-1, proto_string)


if __name__ == "__main__":
    create_nodes()
