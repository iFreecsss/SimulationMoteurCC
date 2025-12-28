import pygame
from pygame.locals import *

from vecteur3d import Vector3D as v
from torseur import Torseur

from math import cos, sin

class Particule:

    def __init__(self, position=v(), init_speed=v(), masse=1.0, name='name', color=None, fixed=False):
        import numpy as np

        self.name = name
        self.masse = masse
        
        if color is None:
            self.color = np.random.rand(3) * 255
        else:
            self.color = color

        self.position = [position]
        self.speed = [init_speed]
        self.acceleration = [v(0,0,0)]
        
        self.forces_ext = []

        self.fixed = fixed

    def applyForce(self, *args):
        for f in args:
            self.forces_ext.append(f)
    
    def simule(self, step):
        total_force = sum(self.forces_ext, v(0,0,0))
        
        if self.masse != 0 and not self.fixed:
            a = total_force * (1.0 / self.masse)
        else:
            a = v(0,0,0)

        vit = self.speed[-1] + a * step
        p = self.position[-1] + .5 * a * step**2 + self.speed[-1] * step

        self.speed.append(vit)
        self.acceleration.append(a)
        self.position.append(p)

        self.forces_ext = []
        
    def getPosition(self):
        return self.position[-1]
    
    def getSpeed(self):
        return self.speed[-1]
    
    def plot(self):
        from matplotlib import pyplot as plt
        xs = [p.x for p in self.position]
        ys = [p.y for p in self.position]
        plt.plot(xs, ys)

    

    def gameDraw(self, screen, scale):
        X = int(scale * self.position[-1].x)
        Y = int(scale * self.position[-1].y)

        VX = int(scale * self.speed[-1].x)
        VY = int(scale * self.speed[-1].y)
        
        size = 5
        
        c = (int(self.color[0]), int(self.color[1]), int(self.color[2]))

            
        pygame.draw.circle(screen,c,(X,Y),size*2,size)
        pygame.draw.line(screen,c,(X,Y),(X+VX,(Y+VY)),size)

class Barre2D:
    def __init__(self, mass=1.0, long=1.0, large=0.1, theta=0.0, centre=v(0,0,0), fixed=False, color='red', nom='barre'):
        # Physique
        self.mass = mass
        self.long = long
        self.large = large
        self.J = (1/12) * self.mass * (self.long**2 + self.large**2)

        self.G = centre
        self.theta = theta
        
        self.vitesse = v(0,0,0)
        self.omega = 0.0
        
        # initialisation du torseur des actions au point G
        self.actions_meca = Torseur(P=self.G, R=v(0,0,0), M=v(0,0,0))

        self.fixed = fixed
        self.color = color
        self.nom = nom

        self.pos = [self.G]
        self.rotation = [self.theta]
        self.ts = [0]

    def getPosition(self):
        return self.G

    def getSpeed(self):
        return self.vitesse

    def applyEffort(self, Force=v(), Torque=v(),Point=0):
        if self.fixed:
            return
        
        self.actions_meca.M = self.actions_meca.M + Torque
        # Vecteur unitaire de la barre
        u_barre = v(cos(self.theta), sin(self.theta), 0)
        
        # Distance depuis le centre G
        dist = (self.long / 2.0) * Point
        
        # P_app = G + dist * u_barre
        P_app = self.G + (u_barre * dist)

        T_effort = Torseur(P=P_app, R=Force, M=v(0,0,0))

        self.actions_meca.P = self.G
        self.actions_meca = self.actions_meca + T_effort


    def simule(self, step):
        if self.fixed:
            self.actions_meca = Torseur(P=self.G, R=v(0,0,0), M=v(0,0,0))
            return

        # Accélération
        sum_F = self.actions_meca.R
        acceleration = sum_F * (1.0 / self.mass)
        
        sum_M_z = self.actions_meca.M.z
        alpha = sum_M_z / self.J
        
        # Vitesse
        self.vitesse = self.vitesse + acceleration * step
        self.omega = self.omega + alpha * step
        
        # Position
        self.G = self.G + self.vitesse * step
        self.theta = self.theta + self.omega * step

        # on remet le torseur à zéro pour les pas suivants
        self.actions_meca = Torseur(P=self.G, R=v(0,0,0), M=v(0,0,0))

        self.pos.append(self.G)
        self.rotation.append(self.theta)
        self.ts.append(self.ts[-1] + step)

    def gameDraw(self, screen, scale):

        u = v(cos(self.theta), sin(self.theta), 0)
        n = v(-sin(self.theta), cos(self.theta), 0)
        
        L2 = self.long / 2.0
        l2 = self.large / 2.0
        
        # Coin 1 : Avant Gauche
        c1 = self.G + (u * L2) + (n * l2)
        # Coin 2 : Arrière Gauche
        c2 = self.G - (u * L2) + (n * l2)
        # Coin 3 : Arrière Droite
        c3 = self.G - (u * L2) - (n * l2)
        # Coin 4 : Avant Droite
        c4 = self.G + (u * L2) - (n * l2)
        
        points_poly = [
            (int(c1.x * scale), int(c1.y * scale)),
            (int(c2.x * scale), int(c2.y * scale)),
            (int(c3.x * scale), int(c3.y * scale)),
            (int(c4.x * scale), int(c4.y * scale))
        ]
        
        c = self.color if not isinstance(self.color, str) else pygame.Color(self.color)
        
        pygame.draw.polygon(screen, c, points_poly)
        pygame.draw.polygon(screen, (0,0,0), points_poly, 2)

        # dessin d'un point au centre de gravité
        cg = (int(self.G.x * scale), int(self.G.y * scale))
        pygame.draw.circle(screen, (0,0,0), cg, 3)

if __name__ == "__main__":

    from multiverse import Univers
    from forces_liaisons import *
    # obligé d'importer le module lui même car sinon l'objet barre2D n'est pas traité car considéré différent
    from particule import *

    uni = Univers(dimensions=(10, 10))
    b = Barre2D(centre=v(5, 5, 0))

    uni.addObjets(b)
    uni.addObjets(Gravity(g=v(0, -9.81, 0)), Viscosity(15, centre_zone=v(5, 2, 0)))
    uni.simulateRealTime()

