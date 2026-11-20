from controller import Supervisor

from simulation import config as c


def create_nodes():
    """
    Creates n_vehicles vehicles in the simulation
    for each vehicle, create an emitter and a receiver in the supervisor
    """

    supervisor = Supervisor()

    root = supervisor.getRoot()
    root_children_field = root.getField("children")
    emitters = "\n".join([f'            Emitter  {{name "supervisor_emitter_{i}"}}' for i in range(c.n_vehicles)])
    receivers = "\n".join([f'            Receiver {{name "supervisor_receiver_{i}"}}' for i in range(c.n_vehicles)])
    
    proto_string = f"""
    DEF WorldSupervisor Robot {{
        supervisor TRUE
        name "WorldSupervisor"
        controller "controller_world_supervisor"
        children [
            {emitters}
            {receivers}
        ]
    }}
    """
    root_children_field.importMFNodeFromString(-1, proto_string)

    for i in range(c.n_vehicles):
        proto_string = f"""
        DEF TT02_{i} TT02_2023b {{
            name "TT02_{i}"
            controller "controller_vehicle_driver"
            lidar_horizontal_resolution {c.lidar_horizontal_resolution}
            camera_horizontal_resolution {c.camera_horizontal_resolution or 1}
        }}
        """
        root_children_field.importMFNodeFromString(-1, proto_string)

    for i in range(c.n_vehicles, c.n_vehicles + c.n_stupid_vehicles):
        proto_string = f"""
        DEF TT02_{i} TT02_2023b {{
            name "TT02_{i}"
            controller "controller_red"
            color 1.0 0.0 0.0
            lidar_horizontal_resolution {c.lidar_horizontal_resolution}
            camera_horizontal_resolution {c.camera_horizontal_resolution or 1}
        }}
        """
        root_children_field.importMFNodeFromString(-1, proto_string)


if __name__ == "__main__":
    create_nodes()
