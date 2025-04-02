from typing import *
import numpy as np
import random
import gymnasium as gym
import time
from threading import Lock
from torch.cuda import is_available

from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.vec_env import SubprocVecEnv, DummyVecEnv

from controller import Supervisor
from checkpoint import Checkpoint
from checkpointmanager import CheckpointManager, checkpoints


S = Supervisor()


class WebotsGymEnvironment(gym.Env):
    """
    One environment for each vehicle

    n: index of the vehicle
    supervisor: the supervisor of the simulation
    """

    def __init__(self, i: int, n_envs: int, n_actions: int, lidar_horizontal_resolution: int, lidar_max_range: float, reset_lock: Lock):
        #print the exported node string
        self.i = i
        if n_envs <= 1:
            self.y = -2.5
        else:
            self.y = -2.5 + (self.i / (n_envs - 1) - 0.5) * 1.2
        self.n_actions = n_actions

        self.lidar_horizontal_resolution = lidar_horizontal_resolution
        self.lidar_max_range = lidar_max_range
        self.n_sensors = 1

        self.checkpoint_manager = CheckpointManager(S, checkpoints)

        basicTimeStep = int(S.getBasicTimeStep())
        self.sensorTime = basicTimeStep // 4

        self.reset_lock = reset_lock
        # virtual time of the last reset

        # negative value so that the first reset is not skipped
        self.last_reset = -1e6
        #print(f"{self.last_reset=}")

        # Emitter
        self.emitter = S.getDevice(f"supervisor_emitter_{i}")
        self.emitter.setChannel(2 * self.i)

        # Receiver
        self.receiver = S.getDevice(f"supervisor_receiver_{i}")
        self.receiver.enable(self.sensorTime)
        self.receiver.setChannel(2 * self.i + 1)

        # Last data received from the car
        self.last_data = np.zeros(self.lidar_horizontal_resolution + self.n_sensors, dtype=np.float32)

        self.translation_field = S.getFromDef(f"TT02_{self.i}").getField("translation") # may cause access issues ...

        self.action_space = gym.spaces.Discrete(n_actions) #actions disponibles
        min = np.zeros(self.n_sensors + self.lidar_horizontal_resolution)
        max = np.ones(self.n_sensors + self.lidar_horizontal_resolution)
        self.observation_space = gym.spaces.Box(min, max, dtype=np.float32) #Etat venant du LIDAR
        print(self.observation_space)
        print(self.observation_space.shape)

    # returns the lidar data of all vehicles
    def observe(self):
        # gets from Receiver
        if self.receiver.getQueueLength() > 0:
            while self.receiver.getQueueLength() > 1:
                self.receiver.nextPacket()
            self.last_data = np.clip(np.frombuffer(self.receiver.getBytes(), dtype=np.float32), 0, self.lidar_max_range)

        return self.last_data

    # reset the gym environment reset
    def reset(self, seed=0):
        global n_envs

        # this has to be done otherwise thec cars will shiver for a while sometimes when respawning and idk why
        if S.getTime() - self.last_reset >= 1:
            #print(self.last_reset, S.getTime() - self.last_reset)
            self.last_reset = S.getTime()

            self.checkpoint_manager.reset()

            INITIAL_trans = [-1, self.y, 0.0391]
            INITIAL_rot = [-0.304369, -0.952554, -8.76035e-05 , 6.97858e-06]

            # WARNING: this is not thread safe
            # This may not be necessary but it's better to be safe than sorry
            with self.reset_lock:
                vehicle = S.getFromDef(f"TT02_{self.i}")
                vehicle.getField("translation").setSFVec3f(INITIAL_trans)
                vehicle.getField("rotation").setSFRotation(INITIAL_rot)
                vehicle.resetPhysics()

        obs = self.observe()
        #super().step()
        info = {}
        return obs, info

    # step function of the gym environment
    def step(self, action):
        #print("Action: ", action)
        steeringAngle = np.linspace(-.3, .3, self.n_actions, dtype=np.float32)[action, None]
        self.emitter.send(steeringAngle.tobytes())

        # we should add a beacon sensor pointing upwards to detect the beacon
        obs = self.observe()
        sensor_data = obs[:self.n_sensors]

        reward = 0
        done = False
        truncated = False

        x, y, _ = self.translation_field.getSFVec3f()
        b_past_checkpoint = self.checkpoint_manager.update(x, y)
        if b_past_checkpoint:
            print("Checkpoint passed")
        b_collided, = sensor_data # unpack sensor data
        # print(f"data received from car {self.i}", obs[:6])
        up = 0 # TODO remove this

        if b_collided:
            # print("Collision détectée")
            reward = -100
            done = True
        elif up > 700: # TODO remove this
            done = False
            # print("Balise passée")
            reward = 100
        else:
            done = False
            reward = 1

        S.step()

        return obs, reward, done, truncated, {}

    #Fonction render de l"environnement GYM
    def render(self, mode="human", close=False):
        pass



#----------------Programme principal--------------------
def main():
    n_envs = 1
    n_actions = 17
    lidar_horizontal_resolution = 512
    lidar_max_range = 12.0
    print("Creating environment")

    # global i
    # i = -1
    # def f():
    #     global i
    #     i += 1
    #     return WebotsGymEnvironment(i, lidar_horizontal_resolution)
    # env = make_vec_env(
    #     f,
    #     n_envs=n_envs,
    #     vec_env_cls=SubprocVecEnv
    # )

    reset_lock = Lock()
    env = DummyVecEnv([lambda i=i: WebotsGymEnvironment(i, n_envs, n_actions, lidar_horizontal_resolution, lidar_max_range, reset_lock) for i in range(n_envs)])

    print("Environment created")
    # check_env(env)

    logdir = "./Webots_tb/"
    #-- , tensorboard_log = logdir -- , tb_log_name = "PPO_voiture_webots"

    #Définition modèle avec paramètre par défaut
    model = PPO("MlpPolicy", env,
        n_steps=256,
        n_epochs=1, # doesn't make sense here
        batch_size=32,
        learning_rate=1e-3,
        verbose=1,
        device="cuda:0" if is_available() else "cpu"
    )

    #Entrainnement
    model.learn(total_timesteps=1e6)

    #Sauvegarde
    model.save("Voiture_autonome_Webots_PPO")

    #del model

    #Chargement des données d"apprentissage
    #model = PPO.load("Voiture_autonome_Webots_PPO")

    obs = env.reset()

    for _ in range(1000000):
        #Prédiction pour séléctionner une action à partir de l"observation
        action, _states = model.predict(obs, deterministic=True)
        obs, reward, done, info = env.step(action)
        if done:
            obs = env.reset()


if __name__ == "__main__":
    main()
