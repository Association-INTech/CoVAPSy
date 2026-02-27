import os
import re

import psutil

import __main__
from webots import webots_path


def get_simulation_rank():
    """
    This function has to be ran from a webots controller.
    Get the simulation rank of the current python process.
    """

    # check if we are in a controller
    if (
        result := re.match(
            os.path.join(
                webots_path, r"controllers/controller(\w+)/controller(\w+)\.py"
            ),
            __main__.__file__,
        )
    ) is None or result.group(1) != result.group(2):
        raise ValueError(
            "get_simulation_rank has to be called inside a webots controller"
        )

    # webots precess launches webots-bin internally who is the process that launches the controllers
    # that's why we need to get the pppid
    proc = psutil.Process(os.getpid())
    parent = proc.parent()
    assert parent is not None
    grandparent = parent.parent()
    assert grandparent is not None
    pppid = str(grandparent.pid)

    # Simulation rank
    with open("/tmp/autotech/simulationranks", "r") as f:
        matches = re.search(
            pppid + r" (\d+)",
            f.read(),
            re.MULTILINE,
        )
        assert matches is not None

    return int(matches.group(1))
