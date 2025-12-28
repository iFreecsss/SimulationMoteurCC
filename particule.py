import pygame
from pygame.locals import *
from vecteur3d import Vector3D as v
from forces_liaisons import *

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


class Bordure:
    pass

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
    from multiverse import Univers

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
    import numpy as np

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