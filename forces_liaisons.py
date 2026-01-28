from types import MethodType
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

    def __init__(self,coefficient,centre_zone=v(50,50,0),rayon_zone=100, name='viscosity',active=True):
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
        self.k = 1e4
        self.c = 1e2
        super().__init__(L0=L0, k=self.k, c=self.c, P0=P0, P2=P2, name=name, active=active)

class Pivot:

    def __init__(self, objet, support=None, pivot_obj=0, pivot_sup=0):
        self.child = objet
        self.parent = support if support is not None else v(0,0,0)
        
        # Positions relatives (-1 à 1 pour les barres)
        self.pos_rel_child = pivot_obj
        self.pos_rel_parent = pivot_sup

    def getPosition(self):
        return self.getParentPosition()

    def getParentPosition(self):
        # si le parent est un vecteur on renvoie directement sa position
        if isinstance(self.parent, v):
            return self.parent
        
        # si c'est une particule on renvoie sa position
        elif hasattr(self.parent, 'is_particle'):
            return self.parent.getPosition()
            
        # si c'est une barre on calcule la position du pivot
        elif hasattr(self.parent, 'is_barre2D'):
            # P = G + (L/2 * pos_rel) * u
            u = v(cos(self.parent.theta), sin(self.parent.theta), 0)
            dist = (self.parent.long / 2.0) * self.pos_rel_parent
            return self.parent.G + (u * dist)
            
        return v(0,0,0)

    def simule(self, step):

        pos_univers = self.getParentPosition()

        if self.child.is_barre2D:
            dist_G = (self.child.long / 2.0) * abs(self.pos_rel_child)
            J_pivot = self.child.J + self.child.mass * (dist_G**2)
            # On transporte le torseur des forces au pivot
            T_action = self.child.actions_meca
            T_action.changePoint(pos_univers)
            
            # PFD en rotation : alpha = M / J
            alpha = T_action.M.z / J_pivot
            
            self.child.omega += alpha * step
            self.child.theta += self.child.omega * step
            
            # Mise à jour de la position du centre de gravité G
            u = v(cos(self.child.theta), sin(self.child.theta), 0)
            dist_G = (self.child.long / 2.0) * self.pos_rel_child
            
            # G = P - dist * u (Attention au signe selon le côté du pivot)
            self.child.G = pos_univers - (u * dist_G)
            
            # Reset des forces et historiques
            self.child.actions_meca = Torseur(P=self.child.G, R=v(0,0,0), M=v(0,0,0))
            self.child.pos.append(self.child.G)
            self.child.rotation.append(self.child.theta)
            
        elif self.child.is_particle:
            self.child.position[-1] = pos_univers
            # On pourrait aussi transmettre la vitesse du parent ici si besoin

    def gameDraw(self, screen, scale):
        import pygame
        pos = self.getParentPosition()
        px, py = int(pos.x * scale), int(pos.y * scale)
        pygame.draw.circle(screen, (0, 0, 0), (px, py), 4)
        if hasattr((self.child, self.parent), 'gameDraw'):
            self.child.gameDraw(screen, scale)
            self.parent.gameDraw(screen, scale)

class PivotMobile(Pivot):

    def __init__(self, barre, particule_support, position_pivot_barre=0):
        super().__init__(barre, particule_support, position_pivot_barre, 0)
        self.support = particule_support

    def simule(self, step):
        self.pos_univers = self.support.getPosition()
        super().simule(step)

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
    import numpy as np

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
        pivot = Pivot(objet=b, support=v(5, 7, 0), pivot_obj=-1, pivot_sup=0)
        g = Gravity(v(0, -9.81, 0))

        uni.addObjets(b, pivot, g)
        
        uni.simulateRealTime()

    run_glissiere()
    run_barre_pendule()

    def run_glissiere_control():
        uni = Univers(name="Glissière avec Contrôle", game=True)
        perle = Particule(position=v(20, 50, 0), masse=2.0, name='Perle')
        gravite = Gravity(v(0, -9.81, 0))
        visc = Viscosity(coefficient=.7, centre_zone=v(50,50,0), rayon_zone=100)
        rail = Glissiere(origine=v(50, 50, 0), targets=[perle], direction=v(1, 0, 0), k=8000, c=100, name='Rail')

        uni.addObjets(gravite, rail, perle, visc)

        from types import MethodType

        def myInteraction(self, events, keys):
            if keys[K_UP]:
                perle.applyForce(v(0, -50, 0))
            if keys[K_DOWN]:
                perle.applyForce(v(0, 50, 0))
            if keys[K_LEFT]:
                perle.applyForce(v(-50, 0, 0))
            if keys[K_RIGHT]:
                perle.applyForce(v(50, 0, 0))

        uni.gameInteraction = MethodType(myInteraction, uni)

        uni.simulateRealTime()

    run_glissiere_control()

    def run_pendule_inverse_manuel():
        
        uni = Univers(name="Pendule Inversé - Test Manuel", game=True)
        chariot = Particule(position=v(50, 50, 0), masse=5.0, name='Chariot', color=(0, 0, 255))
        rail = Glissiere(origine=v(0, 50, 0), direction=v(1, 0, 0), targets=[chariot], k=1e6, c=500)


        longueur_barre = 10.0
        angle_init = np.pi / 2
        offset_G = v(cos(angle_init), sin(angle_init), 0) * (longueur_barre / 2)
        pos_G = chariot.getPosition() + offset_G

        barre = Barre2D(mass=1.0, long=longueur_barre, large=1.0, theta=angle_init, centre=pos_G, color=(200, 50, 50))
        pivot = PivotMobile(barre=barre, particule_support=chariot, position_pivot_barre=-1)

        gravite = Gravity(v(0, -9.81, 0))
        visc = Viscosity(coefficient=5.0, centre_zone=v(50,50,0), rayon_zone=200)

        uni.addObjets(gravite, rail, visc, chariot, barre, pivot)

        def interaction_clavier(self, events, keys):
            force_mag = 1000.0 
            
            if keys[K_LEFT]:
                chariot.applyForce(v(-force_mag, 0, 0))
            if keys[K_RIGHT]:
                chariot.applyForce(v(force_mag, 0, 0))

        uni.gameInteraction = MethodType(interaction_clavier, uni)
        
        uni.simulateRealTime()

    run_pendule_inverse_manuel()