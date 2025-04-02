from typing import List
from checkpoint import Checkpoint
import numpy as np
from controller import Supervisor
import os

np.random.seed(os.getpid())

class CheckpointManager:
    def __init__(self, supervisor: Supervisor, checkpoints: List[Checkpoint], next_checkpoint=None):
        self.supervisor = supervisor
        self.checkpoints = checkpoints
        self.reset(next_checkpoint)
        for checkpoint in self.checkpoints:
            checkpoint.create_vector_2d(self.supervisor)

    def update(self, x=None, y=None):
        """
        Update the next checkpoint if the vehicle has reached the current one
        """
        # if x is None then y is None because of default value rules in python


        if x is None or self.checkpoints[self.next_checkpoint].check_plane(x, y):
            self.next_checkpoint = (self.next_checkpoint + 1) % len(self.checkpoints)
            return True
        return False

    def getAngle(self):
        """
        Get the angle of the next checkpoint
        """
        return self.checkpoints[self.next_checkpoint].theta

    def getTranslation(self):
        """
        Get the translation of the next checkpoint
        """
        chp = self.checkpoints[self.next_checkpoint]
        return [chp.x0, chp.y0, 0.0391]

    def getRotation(self):
        """
        Get the rotation of the next checkpoint
        """
        return [0, 0, 1, self.getAngle()]

    def reset(self, i=None):
        if i:
            self.next_checkpoint = i
        else:
            self.next_checkpoint = np.random.randint(0, len(self.checkpoints) - 1)


checkpoints = [
    [ # piste0.wbt
        Checkpoint(0, -0.314494, -2.47211),
        Checkpoint(0, 1.11162, -2.56708),
        Checkpoint(0.8, 2.54552, -2.27446),
        Checkpoint(1.2, 3.58779, -1.38814),
        Checkpoint(1.57, 3.58016, -0.0800134),
        Checkpoint(2.2, 3.23981, 1.26309),
        Checkpoint(1.57, 2.8261, 1.99783),
        Checkpoint(1.04, 3.18851, 2.71151),
        Checkpoint(1.98, 3.6475, 4.09688),
        Checkpoint(2.5, 3.1775, 4.44688),
        Checkpoint(-3, 2.58692, 4.5394),
        Checkpoint(-2.5, 1.52457, 4.3991),
        Checkpoint(-2.2, 0.659969, 3.57074),
        Checkpoint(-1.9, 0.000799585, 2.90417),
        Checkpoint(-1, 0.0727115, 1.81299),
        Checkpoint(-1, 0.788956, 1.22248),
        Checkpoint(-1.6, 1.24749, 0.288391),
        Checkpoint(-2.5, 0.88749, -0.281609),
        Checkpoint(-3, 0.0789172, -0.557653),
        Checkpoint(2.7, -0.832859, -0.484867),
        Checkpoint(1.8, -1.79723, 0.408769),
        Checkpoint(1.6, -1.7446, 1.3386),
        Checkpoint(2.2, -1.92104, 2.72452),
        Checkpoint(3, -2.96264, 2.96666),
        Checkpoint(-2.2, -4.19027, 2.74619),
        Checkpoint(-1.6, -4.34725, 1.7503),
        Checkpoint(-1.57, -4.26858, 0.259482),
        Checkpoint(-1.4, -4.20936, -1.06968),
        Checkpoint(-0.8, -4.0021, -2.35518),
        Checkpoint(-0.3, -2.89371, -2.49154),
        Checkpoint(0, -2.01029, -2.51669),
    ],
    [ # piste1.wbt
        Checkpoint(1.57, 5.52, -2.25),
        Checkpoint(1.57, 5.52, -1.25),
        Checkpoint(1.57, 5.52, -0.25),
        Checkpoint(1.57, 5.52, 0.75),
        Checkpoint(1.57, 5.52, 1.75),
        Checkpoint(1.57, 5.52, 2.75),
        Checkpoint(1.57, 5.52, 3.75),
        Checkpoint(2.00, 5.52, 4.75),
        Checkpoint(2.72, 4.8, 5.51),
        Checkpoint(3.14, 3.62, 5.51),
        Checkpoint(3.14, 1.62, 5.51),
        Checkpoint(-2.62, 0.08, 5.48),
        Checkpoint(-2.36, -1.00, 4.50),
        Checkpoint(-2.36, -2.00, 3.50),
        Checkpoint(-1.57, -2.49, 2.50),
        Checkpoint(-1.57, -2.49, 1.25),
        Checkpoint(-1.57, -2.49, 0.00),
        Checkpoint(-2.36, -2.62, -1.36),
        Checkpoint(-2.88, -3.65, -1.56),
        Checkpoint(-2.36, -4.34, -1.75),
        Checkpoint(-1.88, -4.57, -2.58),
        Checkpoint(-1.31, -4.52, -3.89),
        Checkpoint(-0.52, -4.05, -4.42),
        Checkpoint(0.00, -3.013, -4.49),
        Checkpoint(0.26, -2.11, -4.43),
        Checkpoint(0.78, -1.28, -3.77),
        Checkpoint(0.78, -0.49, -2.98),
        Checkpoint(1.31, 0.43, -2.01),
        Checkpoint(2.09, 0.42, -1.06),
        Checkpoint(2.09, -0.27, -0.29),
        Checkpoint(1.57, -0.46, 0.48),
        Checkpoint(0.78, -0.12, 1.33),
        Checkpoint(0.78, 0.75, 2.19),
        Checkpoint(0.78, 1.50, 2.95),
        Checkpoint(0.26, 2.25, 3.51),
        Checkpoint(-0.52, 2.94, 3.46),
        Checkpoint(-1.31, 3.42, 2.77),
        Checkpoint(-2.09, 3.42, 1.93),
        Checkpoint(-2.09, 2.72, 1.26),
        Checkpoint(-1.57, 2.54, 0.49),
        Checkpoint(-1.05, 2.82, -0.29),
        Checkpoint(-1.05, 3.34, -0.97),
        Checkpoint(-1.57, 3.51, -1.98),
        Checkpoint(-1.83, 3.42, -2.73),
        Checkpoint(-2.62, 3.05, -3.40),
        Checkpoint(-2.88, 2.23, -3.53),
        Checkpoint(-2.36, 1.63, -3.83),
        Checkpoint(-1.57, 1.50, -4.49),
        Checkpoint(-1.05, 1.61, -5.14),
        Checkpoint(0.00, 2.37, -5.46),
        Checkpoint(0.00, 3.37, -5.49),
        Checkpoint(0.00, 4.37, -5.49),
        Checkpoint(0.00, 4.37, -5.49),
        Checkpoint(0.26, 4.98, -5.41),
        Checkpoint(1.05, 5.43, -5.00),
        Checkpoint(1.57, 5.52, -3.25),
        Checkpoint(1.57, 5.52, -4.25),

    ],
    [ # piste2.wbt

    ]
]
