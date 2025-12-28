import pygame
from pygame.locals import *

from vecteur3d import Vector3D as v
from torseur import Torseur

from math import cos, sin

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

    def setForce(self,objet):
        if self.active:
            distance = (objet.getPosition() - self.centre_zone).mod()
            if distance > self.rayon_zone:
                return
            f = objet.getSpeed() * (-self.coefficient)
            if hasattr(objet, 'applyForce'):
                objet.applyForce(f)
            else:
                objet.applyEffort(f)

    def gameDraw(self, screen, scale):
        X = int(scale * self.centre_zone.x)
        Y = int(scale * self.centre_zone.y)
        R = int(scale * self.rayon_zone)
        
        pygame.draw.circle(screen, 'red', (X, Y), R, 1)

class Gravity:
    def __init__(self, g, name='gravity', active=True):
        self.name = name
        self.g = g
        self.active = active

    def setForce(self, objet):
        if self.active:
            if hasattr(objet, 'applyForce'):
                objet.applyForce(self.g)
            else:
                objet.applyEffort(self.g)

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

    def gameDraw(self, screen, scale):
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

class Pivot:
    def __init__(self, barre, position_pivot_barre=0, position_pivot_univers=v()):

        self.barre = barre
        # c'est le point de la liaison sur la barre
        self.pos_barre = position_pivot_barre
        self.pos_univers = position_pivot_univers

        # distance entre G et le pivot
        self.dist_G_pivot = (self.barre.long / 2.0) * abs(self.pos_barre)

        # moment d'inertie avec Huygens J_pivot = J_barre + m * d^2
        self.J_pivot = self.barre.J + self.barre.mass * (self.dist_G_pivot**2)

    def simule(self, step):
        # A. On récupère le torseur des forces appliquées sur la barre (Gravité, etc.)
        # Ce torseur est actuellement exprimé au point G (défini dans Barre2D)
        T_action = self.barre.actions_meca
        
        # transport du torseur au point de pivot avec BABAR
        T_action.changePoint(self.pos_univers)
        
        alpha = T_action.M.z / self.J_pivot
        
        self.barre.omega += alpha * step
        self.barre.theta += self.barre.omega * step
        
        # E. Contrainte Cinématique : On force la position de G
        # G tourne autour du Pivot.
        # Le vecteur Pivot -> G est de longueur dist_G_pivot et d'angle (theta + pi si pivot à droite)
        
        # vecteur unitaire de la barre
        u = v(cos(self.barre.theta), sin(self.barre.theta), 0)
        
        # Si le pivot est à droite, G est à gauche
        # Si le pivot est à gauche, G est à droite
        if self.pos_barre < 0:
            vector_P_to_G = u * self.dist_G_pivot
        else:
            vector_P_to_G = u * -self.dist_G_pivot

        self.barre.G = self.pos_univers + vector_P_to_G
        
        # vide le torseur de la barre
        self.barre.actions_meca = Torseur(P=self.barre.G, R=v(0,0,0), M=v(0,0,0))
        
        self.barre.pos.append(self.barre.G)
        self.barre.rotation.append(self.barre.theta)
    
    def gameDraw(self, screen, scale):
        import pygame
        px = int(self.pos_univers.x * scale)
        py = int(self.pos_univers.y * scale)
        pygame.draw.circle(screen, (0, 0, 0), (px, py), 5)

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
        pos_barreative = particule.getPosition() - self.origine

        # on regarde la taille du vecteur projeté sur l'axe de la glissière en faisant un produit scalaire
        # entre la direction de la glissière et le vecteur position relatif
        proj_length = pos_barreative ** self.direction
        
        # maintenant qu'on a la taille de la projection, on peut calculer le vecteur projeté sur l'axe de la glissière
        pos_projetee = self.direction * proj_length
        
        # ici on calcule l'écart entre la position actuelle et la position projetée
        # on souhaite que la particule soit sur la glissière, donc on va appliquer une force pour réduire cet écart
        ecart = pos_barreative - pos_projetee
        
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

    def gameDraw(self, screen, scale, draw=False):
        if self.active and draw:
            start = self.origine + self.direction * (self.longueur_visuelle)
            
            p0 = (int(self.origine.x * scale), int(self.origine.y * scale))
            p1 = (int(start.x * scale), int(start.y * scale))
            
            pygame.draw.line(screen, (80, 80, 80), p0, p1, 3)

if __name__ == "__main__":
    from multiverse import Univers
    
    from forces_liaisons import *
    from particule import *

    from math import pi, cos, sin

    def run_glissiere():

        uni = Univers(name="Glissière", game=True)

        perle = Particule(position=v(20, 50, 0), masse=2.0, name='Perle')

        gravite = Gravity(v(0, -9.81, 0))

        rail = Glissiere(origine=v(50, 50, 0), targets=[perle], direction=v(1, -0.5, 0), k=8000, c=100, name='Rail')


        uni.addObjets(gravite, rail, perle)

        
        uni.simulateRealTime()

    def run_barre_pendule():
        uni = Univers(name="Test Pendule Barre", game=True, dimensions=(10, 10))
        
        b = Barre2D(mass=2.0, long=4.0, large=0.2, theta=-pi/4, centre=v(5, 5, 0), nom="Pendule")
        
        pivot = Pivot(barre=b, position_pivot_barre=-1, position_pivot_univers=v(5, 5, 0))
        
        g = Gravity(v(0, -9.81, 0))

        uni.addObjets(b)
        uni.addObjets(g)
        
        def simulation_custom(self):
            g.setForce(b)
            
            pivot.simule(self.step)
            
            self.time.append(self.time[-1] + self.step)

        from types import MethodType
        uni.simulateAll = MethodType(simulation_custom, uni)
        
        def draw_pivot(screen, scale):
            pivot.gameDraw(screen, scale)
        uni.simulateRealTime()
    

    run_glissiere()
    run_barre_pendule()