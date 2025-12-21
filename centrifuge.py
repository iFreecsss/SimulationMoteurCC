import pygame
import numpy as np
from math import cos, sin, sqrt
from vecteur3d import Vector3D as v
from multiverse import Univers
from moteur_cc import MoteurCC
from particule import Particule, Glissiere, SpringDamper, Gravity

class BrasCentrifuge:
    def __init__(self, moteur, glissiere, particule):
        self.moteur = moteur
        self.glissiere = glissiere
        self.particule = particule
        self.active = True
        
        self.glissiere.origine = self.moteur.position

    def setForce(self, p):
        if not self.active: return
        
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
        
        # centre de rotation du moteur
        cx = int(self.moteur.position.x * scale)
        cy = int(self.moteur.position.y * scale)
        pygame.draw.circle(screen, (50, 50, 50), (cx, cy), 10)


if __name__ == "__main__":
    from types import MethodType

    uni = Univers(name="Centrifugeuse", game=True)

    moteur = MoteurCC(position=v(50, 50, 0), name="Moteur Central")

    # particule qui bouge sur le rail
    masse_mobile = Particule(position=v(55, 50, 0), name="Particule")
    
    # ajout de la glissière (position horizontale initialement mais va changer)
    rail_tournant = Glissiere(origine=moteur.position, direction=v(1, 0, 0), k=15000, c=200)

    # ajout d'un ressort en mettant une seconde particule fixe au centre du moteur
    ressort_rappel = SpringDamper(L0=5, k=10, c=0.5, P0=masse_mobile, P2=Particule(position=moteur.position, name="Fixe"), active=True)
    ressort_rappel.P2.fixed = True 

    bras = BrasCentrifuge(moteur, rail_tournant, masse_mobile)

    uni.addMotors(moteur)
    uni.addParticules(masse_mobile)
    uni.addGenerators(bras, ressort_rappel)

    def interaction(self, events, keys):
        m = self.moteurs[0]
        
        # Contrôle de la tension du moteur
        if keys[pygame.K_UP]:
            m.setVoltage(12.0)
        elif keys[pygame.K_DOWN]:
            m.setVoltage(-12.0)
        elif keys[pygame.K_SPACE]:
            m.setVoltage(0.0)
        elif keys[pygame.K_b]:
            m.setVoltage(0.0) 
            m.omega = 0 
            

    uni.gameInteraction = MethodType(interaction, uni)

    
    uni.simulateRealTime()