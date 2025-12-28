import random
import pygame
from pygame.locals import *
import numpy as np
from vecteur3d import Vector3D as v
from multiverse import Univers

class Particule:
    def __init__(self, position=v(), init_speed=v(), masse=1.0, name='name', color=None, fixed=False):
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

            


class Force:
    
    def __init__(self,force=v(),name='force',active=True):
        self.force = force
        self.name = name
        self.active = active

    def setForce(self,particule):
        if self.active:
            particule.applyForce(self.force)

class ForceSelect(Force):
    def __init__(self,force, target_particule, name='force_select'):
        super().__init__(force, name)
        self.target_particule = target_particule

    def setForce(self, particule):
        for target in self.target_particule:
            if particule == target:
                super().setForce(particule)

class Viscosity:

    def __init__(self,coefficient,centre_zone = v(),rayon_zone=1, name='viscosity',active=True):
        self.coefficient = coefficient
        self.name = name
        self.active = active
        self.rayon_zone = rayon_zone
        self.centre_zone = centre_zone

    def setForce(self,particule):
        if self.active:
            distance = (particule.getPosition() - self.centre_zone).mod()
            if distance > self.rayon_zone:
                return
            f = particule.getSpeed() * (-self.coefficient)
            particule.applyForce(f)
    
    def drawArea(self, screen, scale):
        X = int(scale * self.centre_zone.x)
        Y = int(scale * self.centre_zone.y)
        R = int(scale * self.rayon_zone)
        
        pygame.draw.circle(screen, 'red', (X, Y), R, 1)

class Gravity:
    def __init__(self, g, name='gravity', active=True):
        self.name = name
        self.g = g
        self.active = active

    def setForce(self, particule):
        if self.active:
            particule.applyForce(self.g)


class SpringDamper:
    def __init__(self, L0, k, c, P0, P2, name='spring damper', active=True):
        self.L0 = L0
        self.k = k
        self.c = c
        self.P0 = P0 
        self.P2 = P2 
        self.name = name
        self.active = active

    def setForce(self, particule):
        if not self.active:
            return

        if particule == self.P0 or particule == self.P2:
            pos0 = self.P0.getPosition()
            pos2 = self.P2.getPosition()
            
            direction = pos2 - pos0
            L = direction.mod()
            
            if L == 0: return
            
            u = direction * (1.0 / L) 

            v0 = self.P0.getSpeed()
            v2 = self.P2.getSpeed()
            rel_speed = (v2 - v0) ** u
            
            f = self.k * (L - self.L0) + self.c * rel_speed
            
            if particule == self.P0:
                particule.applyForce(u * f)
            else:
                particule.applyForce(u * (-f))

    def drawArea(self, screen, scale):
        if self.active:
            p1 = self.P0.getPosition()
            p2 = self.P2.getPosition()

            start = (int(p1.x * scale), int(p1.y * scale))
            end = (int(p2.x * scale), int(p2.y * scale))

            pygame.draw.line(screen, (255, 0, 0), start, end, 2)

class Fil(SpringDamper):
    def __init__(self, P0, P2, name='fil', active=True):
        L0 = (P2.getPosition() - P0.getPosition()).mod()
        super().__init__(L0=L0, k=5000, c=100, P0=P0, P2=P2, name=name, active=active)


class Glissiere:
    def __init__(self, origine, direction, targets=[], k=1e6, c=1e2, longueur_visuelle=1000, name='glissiere', active=True):

        self.origine = origine
        self.targets = targets
        
        # Normalisation du vecteur directeur
        l = direction.mod()
        if l != 0:
            self.direction = direction * (1.0 / l)
        else:
            self.direction = v(1, 0, 0) # Défaut
            
        self.k = k
        self.c = c
        self.longueur_visuelle = longueur_visuelle
        self.name = name
        self.active = active

    def setForce(self, particule):
        if not self.active:
            return
        if particule not in self.targets:
            return
        # calcul de la position de la particule relative à la glissière
        pos_relative = particule.getPosition() - self.origine

        # on regarde la taille du vecteur projeté sur l'axe de la glissière en faisant un produit scalaire
        # entre la direction de la glissière et le vecteur position relatif
        proj_length = pos_relative ** self.direction
        
        # maintenant qu'on a la taille de la projection, on peut calculer le vecteur projeté sur l'axe de la glissière
        pos_projetee = self.direction * proj_length
        
        # ici on calcule l'écart entre la position actuelle et la position projetée
        # on souhaite que la particule soit sur la glissière, donc on va appliquer une force pour réduire cet écart
        ecart = pos_relative - pos_projetee
        
        # vitesse totale
        vitesse = particule.getSpeed()
        # composante de la vitesse sur l'axe de la glissière
        vitesse_sur_axe = self.direction * (vitesse ** self.direction)
        # composante perpendiculaire (qu'on veut annuler avec l'amortissement)
        vitesse_perp = vitesse - vitesse_sur_axe
        
        # on calcule frappel avec l'écart qui est perpendiculaire à la glissière et qui n'a donc qu'une 
        # composante perpendiculaire à la glissière
        f_rappel = ecart * (-self.k)
        # on calcule famortissement avec la vitesse perpendiculaire à la glissière
        f_amortissement = vitesse_perp * (-self.c)
        
        particule.applyForce(f_rappel + f_amortissement)

    def drawArea(self, screen, scale, draw=False):
        if self.active and draw:
            start = self.origine + self.direction * (self.longueur_visuelle)
            
            p0 = (int(self.origine.x * scale), int(self.origine.y * scale))
            p1 = (int(start.x * scale), int(start.y * scale))
            
            pygame.draw.line(screen, (80, 80, 80), p0, p1, 3)
            

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
    def run_glissiere():
        from types import MethodType

        uni = Univers(name="Glissière", game=True)

        perle = Particule(position=v(20, 50, 0), masse=2.0, name='Perle')
        uni.addParticules(perle)

        gravite = Gravity(v(0, -9.81, 0))

        rail = Glissiere(origine=v(50, 50, 0), targets=[perle], direction=v(1, -0.5, 0), k=8000, c=100, name='Rail')

        def gameInteraction(self, events, keys):
            if keys[pygame.K_SPACE]:
                perle.applyForce(v(50, 0, 0))
            

        uni.addGenerators(gravite, rail)
        uni.gameInteraction = MethodType(gameInteraction, uni)

        
        uni.simulateRealTime()

if __name__ == "__main__":
    from types import MethodType

    uni = Univers(name="Pendule Simple", game=True, dimensions=(50, 50))
    p = Pendule(attache_pos=v(25, 40, 0), longueur=10.0, angle_init=np.pi/4)
    uni.addParticules(p.ancre, p.masse)

    gravite = Gravity(v(0, -9.81, 0))

    def gameInteraction(self, events, keys):
        vitesse_deplacement = 0.1
        if keys[pygame.K_LEFT]:p.ancre.position[-1].x -= vitesse_deplacement
        if keys[pygame.K_RIGHT]:p.ancre.position[-1].x += vitesse_deplacement
        if keys[pygame.K_UP]:p.ancre.position[-1].y += vitesse_deplacement
        if keys[pygame.K_DOWN]:p.ancre.position[-1].y -= vitesse_deplacement


    uni.addGenerators(gravite, p.fil)
    uni.gameInteraction = MethodType(gameInteraction, uni)

    uni.simulateRealTime()