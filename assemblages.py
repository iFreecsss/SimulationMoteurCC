from math import cos, sin

from torseur import *
from vecteur3d import Vector3D as v

from particule import *
from forces_liaisons import *

from control_pid import *
from moteur_cc import *

import pygame
from pygame.locals import *


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

    from multiverse import *

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
            springdamper = SpringDamper(L0=5, k=5, c=5, P0=masse, P2=Particule(position=controlleur.moteur.position, name="Fixe", fixed=True))

        bras = BrasCentrifuge(controlleur, glissiere, masse)

        uni.addObjets(moteur, masse, bras, springdamper, controlleur)

        controlleur.setTarget(target)
        uni.simulateRealTime()

        controlleur.plot()
        plt.show()

        print(controlleur.getTimeResponse())

        bras.plot()
        plt.show()
    
    # run_centrifuge()

class Pendule:
    def __init__(self, attache_pos=v(0, 0, 0), longueur=1.0, angle_init=0.0):
        from math import cos,sin,sqrt

        self.ancre = Particule(fixed=True, position=attache_pos, color=(0, 0, 0))
        self.masse = Particule(position=attache_pos + longueur*v(sin(angle_init), -cos(angle_init), 0), color=(255, 0, 0))
        self.longueur = longueur
        self.angle = [angle_init]
        self.vitesse = []
        self.fil = Fil(P0=self.ancre, P2=self.masse)

    def gameDraw(self, screen, scale):
        self.ancre.gameDraw(screen, scale)
        self.masse.gameDraw(screen, scale)
        self.fil.drawArea(screen, scale)

    def getAngle(self):
        return self.angle[-1]
    
    def simule(self, step):
        self.fil.setForce(self.masse)

        self.angle.append()
        self.vitesse.append(self.masse.getSpeed())


    def plot(self):
        pass

if __name__ == "__main__":

    def run_pendule():
        from types import MethodType
        import numpy as np

        uni = Univers(name="Pendule Simple", game=True, dimensions=(50, 50))
        p = Pendule(attache_pos=v(25, 40, 0), longueur=10.0, angle_init=np.pi/4)
        uni.addObjets(p.ancre, p.masse)

        gravite = Gravity(v(0, -9.81, 0))

        def gameInteraction(self, events, keys):
            vitesse_deplacement = 0.1
            if keys[pygame.K_LEFT]:p.ancre.position[-1].x -= vitesse_deplacement
            if keys[pygame.K_RIGHT]:p.ancre.position[-1].x += vitesse_deplacement
            if keys[pygame.K_UP]:p.ancre.position[-1].y += vitesse_deplacement
            if keys[pygame.K_DOWN]:p.ancre.position[-1].y -= vitesse_deplacement


        uni.addObjets(gravite, p.fil)
        uni.gameInteraction = MethodType(gameInteraction, uni)

        uni.simulateRealTime()

    # run_pendule()

class TurtleBot:
    def __init__(self, position=v(0,0,0), orientation=0, name='TurtleBot', color=None):
        
        self.position = position
        self.orientation = orientation
        self.name = name
        
        if color is None:
            import numpy as np
            self.color = tuple(np.random.randint(0,256,3))
        else:
            self.color = color

        # un peu comme une moto ou un 2 roues en général dans lesquels la roue arrière est motrice et la roue avant est directrice, 
        # on divise les tâches en 2 en utilisant un moteur qui controle la direction et un autre la propulsion
        self.moteur_av = MoteurCC(name="Direction", visc=1e-2) 
        self.moteur_ar = MoteurCC(name="Propulsion", visc=1e-2)

        # paramètres de la roue
        self.rayon_roue = 0.035
        self.facteur_rotation = 0.1

    def setVoltage(self, vg, vd):
        self.moteur_av.setVoltage(vg)
        self.moteur_ar.setVoltage(vd)

    def simule(self, step):
        self.moteur_av.simule(step)
        self.moteur_ar.simule(step)
        
        # on récup la vitesse de chacun des moteurs
        omega_direction = self.moteur_av.getSpeed()
        omega_propulsion = self.moteur_ar.getSpeed()

        # on calcule la vitesse de rotation et de translation de la motobot
        # dans la vraie vie, le déplacement de la moto dépend de la vitesse de rotation du moteur 
        # mais aussi du rayon de sa roue. Permet de passer de rad/s à m/s.
        v_lineaire = omega_propulsion * self.rayon_roue
        # pour la rotation comme le modèle est simplifié, on utilise juste un facteur pour ralentir 
        # la rotation du moteur pour que sa vitesse soit crédible dans la vraie vie.
        v_rotation = omega_direction * self.facteur_rotation

        # on intègre ensuite la position et on met à jour
        self.position.x += v_lineaire * cos(self.orientation) * step
        self.position.y += v_lineaire * sin(self.orientation) * step
        self.orientation += v_rotation * step

    def gameDraw(self, screen, scale):
        import pygame
        
        X = int(self.position.x * scale)
        Y = int(self.position.y * scale)
        
        radius = 15
        
        c = (int(self.color[0]), int(self.color[1]), int(self.color[2]))
        pygame.draw.circle(screen, c, (X, Y), radius)
        
        end_x = X + int(radius * 1.5 * cos(self.orientation))
        end_y = Y + int(radius * 1.5 * sin(self.orientation))
        pygame.draw.line(screen, (0, 0, 0), (X, Y), (end_x, end_y), 3)

if __name__ == "__main__":

    from assemblages import *

    def run_turtlemotobot():
        from types import MethodType

        uni = Univers(name="Moto GP", game=True, dimensions=(5,5))
        
        rob = TurtleBot(position=v(2.5, 2.5, 0), name="MotoBot")
        
        uni.addObjets(rob)

        def control_tension(self, events, keys):
            import pygame

            # on définit les touches qui nous permettent de contrôler le robot en jouant sur sa tension uniquement.
            U_propulsion = 12.0
            U_direction = 12.0 
            
            val_propulsion = 0
            val_direction = 0
            
            if keys[pygame.K_UP]:
                val_propulsion = U_propulsion
            elif keys[pygame.K_DOWN]:
                val_propulsion = -U_propulsion
                
            if keys[pygame.K_LEFT]:
                val_direction = U_direction
            elif keys[pygame.K_RIGHT]:
                val_direction = -U_direction
                
            rob.setVoltage(val_direction, val_propulsion)

        uni.gameInteraction = MethodType(control_tension, uni)

        uni.simulateRealTime()

    run_turtlemotobot()