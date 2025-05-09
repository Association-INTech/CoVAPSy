"""Voiture autonome avec utilisation d'un
LIDAR sur WEBOTS
Auteur: Chrysanthe et Jessica
"""

import numpy as np
import random
import gymnasium as gym
import time

from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.vec_env import SubprocVecEnv

from vehicle import Driver
from controller import Lidar
from controller import Field
from controller import Supervisor


#--------------GYM----------------------------

#Création de l'environnement GYM
class WebotsGymEnvironment(gym.Env):
    def __init__(self):
        #Initialisation du driver
        self.driver = Driver()

        basicTimeStep = int(self.driver.getBasicTimeStep())
        self.sensorTime = basicTimeStep//4

        #Paramètre de la voiture (position, etc..)
        self.voiture_robot = self.driver.getFromDef("vehicle")

        #Lidar
        self.lidar = self.driver.getDevice('lidar')
        self.lidar.enable(self.sensorTime)
        self.lidar.enablePointCloud()

        #Capteur de distance
        self.capteur_avant = self.driver.getDevice('front_center_sensor')
        self.capteur_gauche = self.driver.getDevice('side_left_sensor')
        self.capteur_droite = self.driver.getDevice('side_right_sensor')
        self.capteur_avant.enable(self.sensorTime)
        self.capteur_gauche.enable(self.sensorTime)
        self.capteur_droite.enable(self.sensorTime)

        #Capteur de balise
        self.capteur_balise = self.driver.getDevice('capteur_balise')
        self.capteur_balise.enable(self.sensorTime)

        self.action_space = gym.spaces.Discrete(5) #actions disponibles
        min = np.zeros(self.lidar.getNumberOfPoints())
        max = np.ones(self.lidar.getNumberOfPoints())
        self.observation_space = gym.spaces.Box(min, max, dtype=np.float32) #Etat venant du LIDAR

        self.trans_champs = self.voiture_robot.getField("translation")
        self.rot_champs = self.voiture_robot.getField("rotation") # idk why but if this goes befor Lidar it will not work


    #Vérification de l'état de la voiture
    def observe(self):
        try:
            tableau = self.lidar.getRangeImage()
            #Division par 10 pour que la valeur soient entre 0 et 1
            etat = np.array(tableau, dtype=np.float32)/10
        except: #En cas de non retour lidar
            print("Pas de retour du lidar")
            etat = np.zeros(self.lidar.getNumberOfPoints(), dtype=np.float32)

        return etat

    #Remise à 0 pour l'environnement GYM
    def reset(self, seed=0):
        #self.capteur_avant.disable()
        #self.capteur_gauche.disable()
        #self.capteur_droite.disable()

        #Valeur aléatoire
        x = random.uniform(1.5, 1.65)
        y = random.uniform(3.66, 3.8)

        #Fonction d'initialisation
        INITIAL_trans = [x, y, 0.0195182]
        INITIAL_rot=[-0.000257, 0.000618, 1 , -0.784]
        self.trans_champs.setSFVec3f(INITIAL_trans)
        self.rot_champs.setSFRotation(INITIAL_rot)

        time.sleep(0.3) #Temps de pause après réinitilialisation

        #self.capteur_avant.enable(self.sensorTime)
        #self.capteur_gauche.enable(self.sensorTime)
        #self.capteur_droite.enable(self.sensorTime)
        #Retour état
        obs = self.observe()
        #super().step()
        info = {}
        return obs, info

    #Fonction step de l'environnement GYM
    def step(self, action):
        self.driver.setSteeringAngle([-.4, -.1, 0, .1, .4][action])
        self.driver.setCruisingSpeed(1.0)

        obs = self.observe()

        reward = 0
        done = False
        truncated = False

        avant = self.capteur_avant.getValue()
        gauche = self.capteur_gauche.getValue()
        droite = self.capteur_droite.getValue()
        balise = self.capteur_balise.getValue()

        if avant >= 900 and not(done):
            print("Collision avant")
            reward = -100
            done = True
        elif ((avant >= 854 and gauche >= 896) or (avant >= 696 and gauche >= 910) or gauche >= 937) and not(done):
            print("Collision gauche")
            reward = -100
            done = True
        elif ((avant >= 850 and droite >= 893) or (avant >= 584 and droite >= 910) or droite >= 961) and not(done):
            print("Collision droite")
            reward = -100
            done = True
        elif balise > 700:
            done = False
            print("Balise passée")
            reward = 20
        else:
            done = False
            reward = 0

        self.driver.step()

        return obs, reward, done, truncated, {}

    #Fonction render de l'environnement GYM
    def render(self, mode="human", close=False):
        pass


#----------------Programme principal--------------------
def main():
    n_vehicles = 4
    # env = SubprocVecEnv([lambda: WebotsGymEnvironment() for i in range(n_vehicles)])
    env = WebotsGymEnvironment()
    check_env(env)

    logdir = "./Webots_tb/"
    #-- , tensorboard_log = logdir -- , tb_log_name = "PPO_voiture_webots"

    #Définition modèle avec paramètre par défaut
    model = PPO('MlpPolicy', env,
        n_steps=2048,
        n_epochs=10,
        batch_size=32,
        learning_rate=3e-3,
        verbose=1,
        device=device
    )

    #Entrainnement
    model.learn(total_timesteps=1e6)

    #Sauvegarde
    model.save("Voiture_autonome_Webots_PPO")

    #del model

    #Chargement des données d'apprentissage
    #model = PPO.load("Voiture_autonome_Webots_PPO")

    obs = env.reset()

    for _ in range(1000000):
        #Prédiction pour séléctionner une action à partir de l'observation
        action, _states = model.predict(obs, deterministic=True)
        obs, reward, done, info = env.step(action)
        if done:
            obs = env.reset()


if __name__ == '__main__':
    main()

