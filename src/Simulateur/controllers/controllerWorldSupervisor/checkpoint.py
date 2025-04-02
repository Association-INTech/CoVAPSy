import numpy as np


class Checkpoint:
    def __init__(self, theta, x0, y0):
        self.theta = theta
        self.x0 = x0
        self.y0 = y0
        self.color = (1, 0, 0)

    def create_vector_2d(self, supervisor):
        """
        Draws a 2D vector on the (x, y) plane starting at the given start position,
        extending in the direction specified by angle, with the given length and color.
        """

        length = 0.5

        # Cylinder (shaft)
        supervisor.getRoot().getField("children").importMFNodeFromString(-1, f"""
        Transform {{
            translation {self.x0} {self.y0} 0
            rotation 0 1 0 {-np.pi / 2}
            children [
                Transform {{
                    rotation 1 0 0 {self.theta + np.pi}
                    children [
                        Shape {{
                            geometry Cylinder {{
                                height {length}  # Leave space for the arrowhead
                                radius 0.02
                            }}
                            appearance Appearance {{
                                material Material {{
                                    diffuseColor {self.color[0]} {self.color[1]} {self.color[2]}
                                }}
                            }}
                        }}
                    ]
                }}
            ]
        }}
        """)

        # Cone (arrowhead)
        supervisor.getRoot().getField("children").importMFNodeFromString(-1, f"""
        Transform {{
            translation {self.x0 + np.cos(self.theta)*length/2} {self.y0 + np.sin(self.theta)*length/2} 0
            rotation 0 1 0 {-np.pi / 2}
            children [
                Transform {{
                    rotation 1 0 0 {self.theta + np.pi}
                    children [
                        Shape {{
                            geometry Cone {{
                                height 0.2
                                bottomRadius 0.05
                            }}
                            appearance Appearance {{
                                material Material {{
                                    diffuseColor {self.color[0]} {self.color[1]} {self.color[2]}
                                }}
                            }}
                        }}
                    ]
                }}
            ]
        }}
        """)

    def check_plane(self, x, y):
        """
        checks whether the point (x, y) is on the "positive side"
        of the plane of equation
        cos(theta)(x-x0) + sin(theta)(y-y0) = 0
        which is equivalent to checking
        u . (x, y) >= 0 but with an offset of (x0, y0) where u is a vector orthogonal to the hyperplane
        """
        return np.cos(self.theta)*(x - self.x0) + np.sin(self.theta)*(y - self.y0) >= 0
