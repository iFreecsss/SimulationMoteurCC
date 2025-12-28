from vecteur3d import Vector3D as v
from math import cos, sin, pi
from moteur_cc import MoteurCC
from multiverse import Univers
from types import MethodType

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
    uni = Univers(name="Moto GP", game=True, dimensions=(5,5))
    
    rob = TurtleBot(position=v(2.5, 2.5, 0), name="MotoBot")
    
    uni.addMotors(rob)

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