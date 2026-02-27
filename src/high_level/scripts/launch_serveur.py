# ---------------------------------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------------------------------
import logging

from high_level import Serveur

if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
        handlers=[
            logging.FileHandler("/home/intech/CoVAPSy/covapsy.log"),
            logging.StreamHandler(),
        ],
    )
    log_serveur = logging.getLogger("__main__")
    log_serveur.setLevel(level=logging.DEBUG)

    log_serveur = logging.getLogger("src.HL")
    log_serveur.setLevel(level=logging.DEBUG)

    log_lidar = logging.getLogger("src.HL.actionneur_capteur.Lidar")
    log_lidar.setLevel(level=logging.INFO)

    boot = Serveur()
    boot.main()
