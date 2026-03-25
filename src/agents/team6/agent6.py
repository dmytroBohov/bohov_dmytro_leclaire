import numpy as np
import random
import math

from utils.track_utils import compute_curvature, compute_slope
from agents.kart_agent import KartAgent


class Agent6(KartAgent):
    def __init__(self, env, path_lookahead=3):
        super().__init__(env)
        self.path_lookahead = path_lookahead
        self.agent_positions = []
        self.obs = None
        self.isEnd = False
        self.name = "Bohov Dmytro" # replace with your chosen name
        self.prev_err = 0.0 

    def reset(self):
        self.obs, _ = self.env.reset()
        self.agent_positions = []
        self.prev_err = 0.0

    def endOfTrack(self):
        return self.isEnd

    def choose_action(self, obs):
        speed = math.sqrt(obs["velocity"][0]**2 + obs["velocity"][2]**2) # on calcule la vitesse avec la formule sqrt(x^2 + z^2)
        idx = int(speed / 12.0) + 1 # on prend le nœuds par rapport à la vitesse 

        target_ctr_x = (obs["paths_start"][idx][0] + obs["paths_end"][idx][0]) / 2.0 # on prend l'x entre le début et fin d'un nœud
        target_ctr_z = (obs["paths_start"][idx][2] + obs["paths_end"][idx][2]) / 2.0 # on fait pareil pour le z


        err = math.atan2(target_ctr_x, target_ctr_z) # angle entre x cible et z cible , 
        p_k = 0.8 # le "poids" ou l'importance qu'on donne à l'erreur
        d_k = 1.5 # pareil ici
        drv = err - self.prev_err # on calcule la difference entre l'angle précedante et actuelle
        self.prev_err = err # l'angle actuelle devient l'angle précedent pour le prochain pas de temps
        steer = max(-1.0, min(p_k * err + d_k * drv, 1.0)) # on fait un "clamp" entre les valeurs -1 et 1

        acceleration = 0.5 # on laisse l'accelaration à 50%
        
        action = {
            "acceleration": acceleration,
            "steer": steer,
            "brake": False, # on fait pas de brake
            "drift": False, # on fait pas de drift
            "nitro": False, # on fait pas de nitro
            "rescue":False, # on fait pas de rescue
            "fire": False, # on fait pas de fire
        }
        return action
