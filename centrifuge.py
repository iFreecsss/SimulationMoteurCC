import pygame
import numpy as np
from math import cos, sin, sqrt
from vecteur3d import Vector3D as v
from control_pid import ControlPID_vitesse
from multiverse import Univers
from moteur_cc import MoteurCC
from particule import Particule, Glissiere, SpringDamper, Gravity

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
    
    run_centrifuge()
        