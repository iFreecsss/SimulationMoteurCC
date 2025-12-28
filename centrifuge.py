import pygame
from math import cos, sin, sqrt
from vecteur3d import Vector3D as v
from control_pid import ControlPID_vitesse
from multiverse import Univers
from moteur_cc import MoteurCC
from particule import Particule
from forces_liaisons import *

class BrasCentrifuge:
    def __init__(self, moteur, glissiere, particule):
        self.moteur = moteur.moteur
        self.glissiere = glissiere
        self.particule = particule
        self.active = True

        self.glissiere.origine = self.moteur.position

    def setForce(self, p):
        if not self.active:return
        if p != self.particule:return
        
        theta = self.moteur.theta

        # mise à jour de la direction de la glissière pour qu'elle suive le moteur
        dir_x = cos(theta)
        dir_y = sin(theta)
        self.glissiere.direction = v(dir_x, dir_y, 0)
        
        # application de la contrainte de la glissière sur la particule 
        self.glissiere.setForce(p)
        
        # on utilise la physique plus réaliste pour la charge inertielle
        # Moment d'inertie donné par la particule J = m * r^2
        dist_vec = p.getPosition() - self.moteur.position
        r = dist_vec.mod()
        
        inertie_variable = p.masse * (r**2)
        
        # Plus la particule est loin, plus le moteur aura du mal à accélérer
        self.moteur.setCharge(inertie_charge=inertie_variable)

    def drawArea(self, screen, scale):
        self.glissiere.drawArea(screen, scale)
        self.moteur.gameDraw(screen, scale)
        
        # centre de rotation du moteur
        cx = int(self.moteur.position.x * scale)
        cy = int(self.moteur.position.y * scale)
        pygame.draw.circle(screen, (50, 50, 50), (cx, cy), 10)

    
    def plot(self):
        import matplotlib.pyplot as plt
        
        ds = []
        for p in self.particule.position:
            d = p - self.moteur.position
            ds.append(d.mod())

        ys = self.moteur.speed_history

        plt.figure(figsize=(20, 30))
        plt.plot(ys, ds)

if __name__ == "__main__":
    
    def run_centrifuge(target=2.0, moteur=None, masse=None, glissiere=None, springdamper=None, controlleur=None):
        import matplotlib.pyplot as plt

        uni = Univers(name="Centrifugeuse", game=True)

        if moteur is None:
            moteur = MoteurCC(couple=0.1, visc=0.001, position=v(50, 50, 0), name="Moteur Central")

        if controlleur is None:
            controlleur = ControlPID_vitesse(moteur, 50, 0.1, 0.01)

        # particule qui bouge sur le rail
        if masse is None:
            masse = Particule(masse=1, position=v(55, 50, 0), name="Particule")

        # ajout de la glissière (position horizontale initialement mais va changer)
        if glissiere is None:
            glissiere = Glissiere(origine=controlleur.moteur.position, k=1e5, c=2e2, targets=[masse], direction=v(1, 0, 0))

        # ajout d'un ressort en mettant une seconde particule fixe au centre du moteur
        if springdamper is None:
            springdamper = SpringDamper(L0=5, k=5, c=5, P0=masse, P2=Particule(position=controlleur.moteur.position, name="Fixe", fixed=True), active=True)

        bras = BrasCentrifuge(controlleur, glissiere, masse)

        uni.addMotors(moteur)
        uni.addParticules(masse)
        uni.addGenerators(bras, springdamper)
        uni.addControlleurs(controlleur)

        controlleur.setTarget(target)
        uni.simulateRealTime()
        controlleur.plot()
        plt.show()
        print(controlleur.getTimeResponse())
        bras.plot()
        plt.show()
    
    # run_centrifuge()

from torseur import Torseur

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
    uni = Univers(dimensions=(10, 10))
    b = Barre2D(centre=v(5, 5, 0))
    uni.addObjects(b)
    uni.addGenerators(Gravity(g=v(0, -9.81, 0)), Viscosity(15, centre_zone=v(5, 2, 0)))
    uni.simulateRealTime()


