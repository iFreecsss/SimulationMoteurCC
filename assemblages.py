from math import cos, pi, sin, atan2, radians, sqrt
import string
import matplotlib.pyplot as plt

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
    
    #run_centrifuge()

class Pendule:
    def __init__(self, attache_pos=v(0, 0, 0), longueur=1.0, angle_init=0.0):
        
        self.ancre = Particule(fixed=True, position=attache_pos, color=(0, 0, 0))
        pos_relative = longueur * v(sin(angle_init), -cos(angle_init), 0)
        self.masse = Particule(masse=1.0, position=attache_pos + pos_relative, color=(255, 0, 0))

        self.longueur = longueur
        
        self.angle = [angle_init]
        self.vitesse = [0.0]
        self.temps = [0.0]
        
        self.fil = Fil(P0=self.ancre, P2=self.masse)

    def gameDraw(self, screen, scale):
        self.ancre.gameDraw(screen, scale)
        self.masse.gameDraw(screen, scale)
        self.fil.gameDraw(screen, scale)
    
    def simule(self, step):
        
        self.fil.setForce(self.masse)
        
        delta = self.masse.position[-1] - self.ancre.position[-1]
        
        theta_actuel = atan2(delta.x, -delta.y)
        
        self.angle.append(theta_actuel)

        self.masse.simule(step)
        
        speed = self.masse.getSpeed()
        self.vitesse.append(speed)
        self.temps.append(self.temps[-1] + step)

    def applyEffort(self, force_vector):
        self.masse.applyForce(force_vector)
        
    def getPosition(self):
        return self.masse.position[-1]

    def plot(self):
        pass

if __name__ == "__main__":
    from assemblages import *
    from multiverse import *

    def run_pendule():
        from types import MethodType
        import numpy as np

        uni = Univers(name="Pendule Simple", game=True, dimensions=(10, 10))
        p = Pendule(attache_pos=v(5, 5, 0), longueur=1.0, angle_init=-np.pi/4)
        gravite = Gravity(v(0, -9.81, 0))

        uni.addObjets(gravite, p.ancre, p.masse, p)
        uni.simulateRealTime()
        
        p.plot()

    #run_pendule()

class PenduleBarre2D:
    def __init__(self, largeur_barre=0.1, longueur_barre=1.0, angle_init=0.0, centre_barre=v(), pos_piv_barre=-1, pos_piv_uni=v(5,5,0)):

        self.barre = Barre2D(mass=1.0, long=longueur_barre, large=largeur_barre, theta=angle_init, centre=centre_barre)
        self.pivot = Pivot(objet=self.barre, support=pos_piv_uni, pivot_obj=pos_piv_barre, pivot_sup=0)
        
    def gameDraw(self, screen, scale):
        self.barre.gameDraw(screen, scale)
        self.pivot.gameDraw(screen, scale)
    
    def simule(self, step):
        self.pivot.simule(step)

    def applyEffort(self, *args, **kwargs):
        self.barre.applyEffort(*args, **kwargs)
        
    def getPosition(self):
        return self.barre.getPosition()

if __name__ == "__main__":

    from assemblages import *

    def run_pendule_barre():
        from types import MethodType
        import numpy as np

        uni = Univers(name="Pendule Barre", game=True, dimensions=(10, 10))

        pb = PenduleBarre2D(angle_init=-np.pi/4)
        g = Gravity(v(0, -9.81, 0))

        uni.addObjets(pb, g)
        uni.simulateRealTime()

    #run_pendule_barre()

    def run_both():
        import numpy as np

        L0 = 1.0

        uni = Univers(name="Pendule Simple", game=True, dimensions=(10, 10))
        p = Pendule(attache_pos=v(1.5, 5, 0), longueur=L0, angle_init=np.pi/4)
        gravite = Gravity(v(0, -9.81, 0))
        pb = PenduleBarre2D(pos_piv_uni=v(3.5, 5, 0), longueur_barre=1.5*L0, largeur_barre=.1, angle_init=-np.pi/4)

        uni.addObjets(gravite, p, pb)
        uni.simulateRealTime()

    #run_both()

class TurtleBot:
    def __init__(self, P0=v(0,0,0), R0=0, name='TurtleBot', color=(0, 255, 0)):
        
        self.position = P0
        self.orientation = R0
        self.pose = [(P0, R0)]
        self.name = name
        self.color = color
        
        # intialisation des vitesses
        self.speedTrans = 0.0
        self.speedRot = 0.0
        
        # Paramètres max
        self.speedTransMax = 5.0
        self.speedRotMax = 5.0 

    def simule(self, step):
        self.orientation += self.speedRot * step
        
        dx = self.speedTrans * cos(self.orientation) * step
        dy = self.speedTrans * sin(self.orientation) * step
        self.position = self.position + v(dx, dy, 0)
        
        self.pose.append((self.position, self.orientation))

    def gameDraw(self, screen, scale):
        
        X = int(scale * self.position.x)
        Y = int(scale * self.position.y)
        
        pygame.draw.circle(screen, self.color, (X, Y), 15)
        
        end_x = X + int(20 * cos(self.orientation))
        end_y = Y + int(20 * sin(self.orientation))
        pygame.draw.line(screen, (0,0,0), (X, Y), (end_x, end_y), 3)

if __name__ == "__main__":
    from multiverse import Univers, v
    from control_pid import ControlPID_Turtle
    from types import MethodType

    MODE = "AUTO"
    #MODE = "MANUEL"

    def run_turtle_final():
        uni = Univers(name="TurtleBot Final", game=True, dimensions=(10, 10))
        rob = TurtleBot(P0=v(5, 5, 0), name="Bot")
        
        if MODE == "MANUEL":
            uni.addObjets(rob)
            
            def interact_manuel(self, events, keys):
                
                rob.speedTrans = 0
                rob.speedRot = 0
                
                if keys[K_UP]:rob.speedTrans = 2.0
                if keys[K_DOWN]:rob.speedTrans = -2.0
                if keys[K_LEFT]:rob.speedRot = 3.0
                if keys[K_RIGHT]:rob.speedRot = -3.0
            
            uni.gameInteraction = MethodType(interact_manuel, uni)

        elif MODE == "AUTO":
            
            # On ajoute le contrôleur dédié (voir si faut régler les gains)
            ctrl = ControlPID_Turtle(rob, Kp_lin=2.0, Ki_lin=1.0, Kd_lin=0.5, Kp_ang=5.0, Ki_ang=1.0, Kd_ang=0.5)
            
            uni.target = v(2, 2, 0)
            visu = Particule(position=uni.target, color=(255,0,0), masse=0)
            ctrl.setTarget(uni.target)
            
            uni.addObjets(rob, ctrl, visu)

            def interact_auto(self, events, keys):
                for e in events:
                    if e.type == MOUSEBUTTONDOWN:
                        mx, my = pygame.mouse.get_pos()
                        px = mx / self.scale
                        py = (self.gameDimensions[1] - my) / self.scale
                        
                        uni.target = v(px, py, 0)
                        visu.position[-1] = uni.target
                        ctrl.setTarget(uni.target)
            
            uni.gameInteraction = MethodType(interact_auto, uni)

        uni.simulateRealTime()

    #run_turtle_final()

class TurtleMotoBot:
    def __init__(self, position=v(0,0,0), orientation=0, name='TurtleBot', color=(0,255,0)):
        
        self.position = position
        self.orientation = orientation
        self.name = name
        
        self.color = color

        # un peu comme une moto ou un 2 roues en général dans lesquels la roue arrière est motrice et la roue avant est directrice, 
        # on divise les tâches en 2 en utilisant un moteur qui controle la direction et un autre la propulsion
        self.moteur_av = MoteurCC(name="Direction", visc=1e-2) 
        self.moteur_ar = MoteurCC(name="Propulsion", visc=1e-2)

        # paramètres de la roue
        self.rayon_roue = 0.035
        self.facteur_rotation = 0.1
        v_max = self.rayon_roue / self.facteur_rotation * self.moteur_ar.Um_max
        print(f"Vitesse max théorique : {v_max:.2f} m/s")

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

    def target_fonction(self, target):
        from math import atan2, pi, cos

        target_dir = target - self.position
        dist = target_dir.mod()

        if dist < 0.01:
            return {
                'traj': 0.0,
                'speed': 0.0
            }
        
        angle_desire = atan2(target_dir.y, target_dir.x)
        
        orientation_actuelle = self.orientation
        diff = angle_desire - orientation_actuelle
        while diff > pi: diff -= 2*pi
        while diff < -pi: diff += 2*pi

        consigne_cap = orientation_actuelle + diff

        alignement = max(0, cos(diff))
        
        v_consigne = dist * 1e3 * alignement
        
        return {'traj': consigne_cap,'speed': v_consigne}

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
    from multiverse import Univers, v
    from assemblages import TurtleMotoBot # Importe ta nouvelle classe
    from control_pid import ControlPID_position, ControlPID_vitesse, Pilote
    import pygame
    from types import MethodType

    def run_turtle_moto_bot():

        uni = Univers(name="Moto Generique", game=True, dimensions=(10, 10))
        rob = TurtleMotoBot(position=v(2, 2, 0), name="MotoBot")
        
        pid_direction = ControlPID_position(moteur=rob.moteur_av,K_P=100.0, K_I=15.0, K_D=50.0,getPosition=lambda: rob.orientation,getSpeed=lambda: rob.moteur_av.getSpeed() * rob.facteur_rotation)
        pid_propulsion = ControlPID_vitesse(moteur=rob.moteur_ar,K_P=30.0, K_I=5.0, K_D=100.0)
        pid_propulsion.anti_windup = True

        pilote = Pilote(rob)
        
        pilote.addController('traj', pid_direction)
        pilote.addController('speed', pid_propulsion)
        
        uni.target = v(5, 5, 0)
        visu_target = Particule(position=uni.target, color=(255, 0, 0), masse=0)
        pilote.setTarget(uni.target)

        uni.addObjets(rob, pilote, visu_target)

        def interaction(self, events, keys):
            # Changement de cible à la souris
            for event in events:
                if event.type == pygame.MOUSEBUTTONDOWN:
                    mx, my = pygame.mouse.get_pos()
                    px = mx / self.scale
                    py = (self.gameDimensions[1] - my) / self.scale
                    uni.target = v(px, py, 0)
                    visu_target.position[-1] = uni.target
                    pilote.setTarget(uni.target)

        uni.gameInteraction = MethodType(interaction, uni)
        uni.simulateRealTime()

    #run_turtle_moto_bot()

class PenduleInverse:
    def __init__(self, pos_chariot=v(50, 50, 0), longueur_barre=10.0, largeur_barre=1.0, angle_init=0.0):
        self.longueur_barre = longueur_barre
        self.largeur_barre = largeur_barre
        self.angle = angle_init

        self.pos_chariot = pos_chariot
        self.chariot = Particule(position=pos_chariot, masse=5.0, name='Chariot', color=(0, 0, 255))

        G_pos = v(cos(self.angle), sin(self.angle), 0) * (self.longueur_barre / 2)
        G = self.chariot.getPosition() + G_pos

        self.pos_glissiere = self.pos_chariot

        self.barre = Barre2D(mass=1.0, long=self.longueur_barre, large=self.largeur_barre, theta=self.angle, centre=G, color=(200, 50, 50))
        self.pivot = PivotMobile(barre=self.barre, particule_support=self.chariot, position_pivot_barre=-1)
        self.rail = Glissiere(origine=self.pos_glissiere, direction=v(1, 0, 0), targets=[self.chariot], k=1e6, c=500)

        self.g = Gravity(v(0, -9.81, 0))
        self.coef_visc = 1.0
        self.visc = Viscosity(coefficient=self.coef_visc, centre_zone=pos_chariot, rayon_zone=200)

    def simule(self, step):
        
        self.g.setForce(self.chariot)
        self.g.setForce(self.barre)
        self.rail.setForce(self.chariot)
        self.visc.setForce(self.chariot)
        # self.visc.setForce(self.barre)

        self.chariot.simule(step)

        # accélération qu'on vient de donner au chariot (que ce soit avec commandes ou controleur)
        accel_chariot = self.chariot.acceleration[-1]

        # On transmet cette accélération à la barre c'est grâce à ça que la barre peut retourner en position pi/2
        # F_inertie = -m_barre * a_chariot
        force_inertie = accel_chariot * (-self.barre.mass)
        self.barre.applyEffort(Force=force_inertie, Point=0)

        self.pivot.simule(step)

    def gameDraw(self, screen, scale):

        self.rail.gameDraw(screen, scale)
        self.chariot.gameDraw(screen, scale)
        self.barre.gameDraw(screen, scale)
        self.pivot.gameDraw(screen, scale)

    # les trois méthodes suivantes sont juste pour que le PID puisse accéder aux informations de l'état
    # je pourrais créer une autre classe controlPID pour gérer cela et que ce soit cohérent en terme de nom de variables
    # mais comme à terme le but est de controler avec un moteurCC à la base, je préfère garder cela ici

    def getPosition(self):
        return self.barre.theta

    def getSpeed(self):
        return self.barre.omega

    def setVoltage(self, commande):
        self.chariot.applyForce(v(commande, 0, 0))
        
if __name__ == "__main__":
    from multiverse import Univers, v
    from pygame.locals import *
    from types import MethodType
    import numpy as np

    def run_pendule_class():
        
        uni = Univers(name="Pendule Inversé - Classe", game=True, dimensions=(50, 50))

        p = PenduleInverse(pos_chariot=v(25, 25, 0), angle_init=np.pi/2 + 0.05)
        p.barre.mass = 0.1
        uni.addObjets(p)

        def interaction_clavier(self, events, keys):
            force_mag = 10000.0
            
            if keys[K_LEFT]:
                p.chariot.applyForce(v(-force_mag, 0, 0))
            if keys[K_RIGHT]:
                p.chariot.applyForce(v(force_mag, 0, 0))

        uni.gameInteraction = MethodType(interaction_clavier, uni)
        
        uni.simulateRealTime()

    #run_pendule_class()

if __name__ == "__main__":
    from multiverse import Univers, v
    from control_pid import ControlPID_position
    from assemblages import PenduleInverse 
    import numpy as np

    def run_stabilisation_pid_standard():
        uni = Univers(name="Pendule Inversé - PID Standard", game=True, dimensions=(10, 10))
    
        p = PenduleInverse(pos_chariot=v(5, 2, 0), angle_init=np.pi/2, longueur_barre=1.0, largeur_barre=0.1)
        # on veut un K_P élevé de sorte que le système réagisse vite mais aussi un K_D assez important pour amortir la rechute du pendule
        # et contrôler les oscillations
        pid = ControlPID_position(moteur=p, K_P=2000, K_I=0, K_D=500)
        
        pid.setTarget(np.pi/2)

        uni.addObjets(p, pid)
        uni.force_pichenette = 10000.0

        def interaction_clavier(self, events, keys):
            
            for event in events:
                if event.type == pygame.KEYDOWN:
                    
                    if event.key == pygame.K_UP:
                        self.force_pichenette += 1000.0
                        print(f"Puissance : {self.force_pichenette} N")
                        
                    if event.key == pygame.K_DOWN:
                        self.force_pichenette -= 1000.0
                        if self.force_pichenette < 0: self.force_pichenette = 0
                        print(f"Puissance : {self.force_pichenette} N")

                    if event.key == pygame.K_LEFT:
                        p.chariot.applyForce(v(-self.force_pichenette, 0, 0))
                        print(f"Pichenette !")
                        
                    if event.key == pygame.K_RIGHT:
                        p.chariot.applyForce(v(self.force_pichenette, 0, 0))
                        print(f"Pichenette !")

                    if event.key == pygame.K_r:
                        p.chariot.position[-1] = v(5, 2, 0)
                        p.chariot.speed[-1] = v(0, 0, 0)
                        p.barre.theta = np.pi/2
                        p.barre.omega = 0.0
                    
                    if event.key == pygame.K_g:
                        self.force_pichenette = 3e5
                        print(f"Mode giga pichenette ! Puissance : {self.force_pichenette} N")
                        

        uni.gameInteraction = MethodType(interaction_clavier, uni)
        uni.simulateRealTime()

    #run_stabilisation_pid_standard()

class PenduleInverseMotorise(PenduleInverse):
    def __init__(self, pos_chariot=v(50, 50, 0), longueur_barre=1.0, largeur_barre=0.1, angle_init=0.0):
        super().__init__(pos_chariot, longueur_barre, largeur_barre, angle_init)
        
        # Moteur :
        # on imagine un moteur qui fait tourner une roue dentée de rayon 5cm qui va entraîner le chariot car on ne peut pas fixer 
        # le moteur directement sur le chariot sinon on ferait juste tourner la particule sur elle même sans la déplacer
        self.moteur = MoteurCC(resistance=1.0, inductance=0.001, fcem=0.5, couple=0.5, name="Moteur Traction")
        self.moteur.position = pos_chariot
        self.rayon_roue = 0.05

    def simule(self, step):
        self.g.setForce(self.chariot)
        self.g.setForce(self.barre)
        self.rail.setForce(self.chariot)
        self.visc.setForce(self.chariot)

        vitesse_x = self.chariot.getSpeed().x
        self.moteur.omega = vitesse_x / self.rayon_roue
        self.moteur.simule(step)
        
        # conversion du couple moteur en force (F = C / r et C = Kc * i)
        couple_moteur = self.moteur.Kc * self.moteur.i
        force_traction = couple_moteur / self.rayon_roue


        # on a fait le plus dur maintenant c'est comme pour le pendule inversé classique on a une force et on l'applique
        self.chariot.applyForce(v(force_traction, 0, 0))
        self.chariot.simule(step)

        accel_chariot = self.chariot.acceleration[-1]
        force_inertie = accel_chariot * (-self.barre.mass)
        self.barre.applyEffort(Force=force_inertie, Point=0)

        self.pivot.simule(step)
        
        self.moteur.position = self.chariot.getPosition()

if __name__ == "__main__":
    from multiverse import Univers, v
    from control_pid import ControlPID_position
    from assemblages import PenduleInverseMotorise 
    import numpy as np
    from types import MethodType
    import pygame

    def run_stabilisation_pid_motorise():
        uni = Univers(name="Pendule Inversé - PID Motorisé", game=True, dimensions=(10, 10))
    
        p = PenduleInverseMotorise(pos_chariot=v(5, 2, 0), angle_init=np.pi/2+0.1, longueur_barre=1.0, largeur_barre=0.1)
        pid = ControlPID_position(moteur=p.moteur, K_P=500, K_I=0, K_D=100, getPosition=p.barre.getAngle, getSpeed=p.barre.getSpeed)
        
        pid.setTarget(np.pi/2)

        uni.addObjets(p, pid)
        uni.force_pichenette = 10000.0

        def interaction_clavier(self, events, keys):
            
            for event in events:
                if event.type == pygame.KEYDOWN:
                    
                    if event.key == pygame.K_UP:
                        self.force_pichenette += 1000.0
                        print(f"Puissance : {self.force_pichenette} N")
                        
                    if event.key == pygame.K_DOWN:
                        self.force_pichenette -= 1000.0
                        if self.force_pichenette < 0: self.force_pichenette = 0
                        print(f"Puissance : {self.force_pichenette} N")

                    if event.key == pygame.K_LEFT:
                        p.chariot.applyForce(v(-self.force_pichenette, 0, 0))
                        print(f"Pichenette !")
                        
                    if event.key == pygame.K_RIGHT:
                        p.chariot.applyForce(v(self.force_pichenette, 0, 0))
                        print(f"Pichenette !")

                    if event.key == pygame.K_r:
                        p.chariot.position[-1] = v(5, 2, 0)
                        p.chariot.speed[-1] = v(0, 0, 0)
                        p.barre.theta = np.pi/2
                        p.barre.omega = 0.0
                    
                    if event.key == pygame.K_g:
                        self.force_pichenette = 3e4
                        print(f"Mode giga pichenette ! Puissance : {self.force_pichenette} N")
        uni.gameInteraction = MethodType(interaction_clavier, uni)
        uni.simulateRealTime()

    #run_stabilisation_pid_motorise()

class Bras1R:
    def __init__(self, longueur=1.0, masse=1.0, angle_init=0.0):
        self.longueur = longueur
        self.angle_init = angle_init
        
        G = v(cos(self.angle_init), sin(self.angle_init), 0) * (self.longueur / 2)
        
        self.barre = Barre2D(mass=masse, long=self.longueur, large=0.1, theta=self.angle_init, centre=G, color='blue', nom="Bras 1R")
        self.pivot = Pivot(objet=self.barre, support=v(2,2,0), pivot_obj=-1, pivot_sup=0)
        
        self.moteur = MoteurCC(resistance=1.0, couple=2.0, name="Moteur Coude")
        
        self.g = Gravity(v(0, -9.81, 0))

    def simule(self, step):

        self.g.setForce(self.barre)
        self.moteur.omega = self.barre.omega
        self.moteur.simule(step)
        couple_moteur = self.moteur.Kc * self.moteur.i
        
        self.barre.applyEffort(Torque=v(0, 0, couple_moteur))
        
        self.pivot.simule(step)

    def gameDraw(self, screen, scale):
        self.barre.gameDraw(screen, scale)
        self.pivot.gameDraw(screen, scale)
        pygame.draw.circle(screen, (100, 100, 100), (int(0), int(0)), 10)

if __name__ == "__main__":
    from types import MethodType

    def bras1R_BF():
        uni = Univers(name="Bras 1R Asservi", game=True, dimensions=(10, 10)) 
        
        bras = Bras1R(longueur=2.0, masse=1.0, angle_init=-pi/2)
        
        pid = ControlPID_position(moteur=bras.moteur,K_P=5000.0, K_I=100.0, K_D=1000.0,getPosition=lambda: bras.barre.theta,getSpeed=lambda: bras.barre.omega)
        pid.modulo = True

        part_target = Particule(position=v(5, 5, 0), color=(255, 0, 0), masse=0, name="Cible")

        uni.addObjets(bras, pid, part_target)

        def interaction_particule(self, events, keys):
            vitesse_deplacement = 0.1
            
            if keys[K_UP]:    part_target.getPosition().y += vitesse_deplacement
            if keys[K_DOWN]:  part_target.getPosition().y -= vitesse_deplacement
            if keys[K_LEFT]:  part_target.getPosition().x -= vitesse_deplacement
            if keys[K_RIGHT]: part_target.getPosition().x += vitesse_deplacement
            
            direction = part_target.getPosition() - bras.pivot.getPosition()
            angle_desire = atan2(direction.y, direction.x)
            pid.setTarget(angle_desire)

        uni.gameInteraction = MethodType(interaction_particule, uni)
        uni.simulateRealTime()
        plt.figure(figsize=(10, 6))
        
        # On utilise la méthode du PID comme demandé
        pid.plot()
        
        plt.title("Position angulaire du Bras au cours du temps")
        plt.xlabel("Temps (s)")
        plt.ylabel("Angle (rad)")
        plt.grid(True)
        plt.show()
    #bras1R_BF()

if __name__ == "__main__":
    from types import MethodType
    def bras1R_BF():
        from multiverse import Univers, InputBox
        import math

        uni = Univers(name="Bras 1R Asservi", game=True, dimensions=(10, 10)) 
        bras = Bras1R(longueur=2.0, masse=1.0, angle_init=-math.pi/2)
        
        pid = ControlPID_position(moteur=bras.moteur,K_P=5000.0, K_I=100.0, K_D=1000.0,getPosition=lambda: bras.barre.theta,getSpeed=lambda: bras.barre.omega)
        pid.modulo = True
        
        def nouvel_angle(text):
            try:
                angle_deg = float(text)
                angle_rad = radians(angle_deg)
                pid.setTarget(angle_rad)
            except ValueError:
                pass

        box_angle = InputBox(10, 80, 140, 32, text='0', title='Angle (en °) : ', callback=nouvel_angle)
        uni.addObjets(bras, pid, box_angle)
        
        uni.simulateRealTime()
        
    #bras1R_BF()

class Bras2R:
    def __init__(self, l1=1.0, l2=1.0, m1=1.0, m2=1.0, angle1=0.0, angle2=0.0):
        self.l1 = l1
        self.l2 = l2
        
        G1 = v(cos(angle1), sin(angle1), 0) * (l1 / 2)
        self.barre1 = Barre2D(mass=m1, long=l1, large=0.1, theta=angle1, centre=G1, color='red', nom="Bras 1")
        self.pivot1 = Pivot(objet=self.barre1, support=v(2, 2, 0), pivot_obj=-1, pivot_sup=0)
        self.moteur1 = MoteurCC(resistance=1.0, couple=5.0, name="Moteur Epaule")

        pos_coude = self.pivot1.getPosition() + v(cos(angle1), sin(angle1), 0) * l1
        G2 = pos_coude + v(cos(angle2), sin(angle2), 0) * (l2 / 2)

        self.barre2 = Barre2D(mass=m2, long=l2, large=0.1, theta=angle2, centre=G2, color='blue', nom="Bras 2")
        
        self.pivot2 = Pivot(objet=self.barre2, support=self.barre1, pivot_obj=-1, pivot_sup=1)
        self.moteur2 = MoteurCC(resistance=1.0, couple=2.0, name="Moteur Coude")
        
        self.g = Gravity(v(0, -9.81, 0))

    def simule(self, step):
        self.g.setForce(self.barre1)
        self.g.setForce(self.barre2)

        self.moteur1.omega = self.barre1.omega
        self.moteur1.simule(step)
        couple1 = self.moteur1.Kc * self.moteur1.i

        # La vitesse vue par le moteur est la différence entre les deux barres
        self.moteur2.omega = self.barre2.omega - self.barre1.omega
        self.moteur2.simule(step)
        couple2 = self.moteur2.Kc * self.moteur2.i
        
        # Le moteur 1 agit sur le bras 1 uniquement 
        # MAIS le moteur 2 agit sur les 2 !
        total_torque_barre1 = couple1 - couple2
        self.barre1.applyEffort(Torque=v(0, 0, total_torque_barre1))
        self.barre2.applyEffort(Torque=v(0, 0, couple2))

        # on fait comme pour le bras 1R
        self.pivot1.simule(step)

        # position du pivot 2 attaché au bras 1 dépend donc de l'angle actuel du bras 1
        u1 = v(cos(self.barre1.theta), sin(self.barre1.theta), 0)
        pos_coude_actuelle = self.pivot1.getPosition() + (u1 * self.barre1.long)
        
        self.pivot2.support = pos_coude_actuelle
        self.pivot2.simule(step)
    
    def mgi(self, position):
        from math import atan2, sqrt, pi

        if not self.zone_atteignable(position):
            return [(0,0)]

        px, py, pz = position.x-self.pivot1.getPosition().x, position.y-self.pivot1.getPosition().y, position.z-self.pivot1.getPosition().z
        L1, L2 = self.l1, self.l2
        
        C2 = (px**2 + py**2 - L1**2 - L2**2) / (2 * L1 * L2)
        S2 = sqrt(1 - C2**2)
        q2s = [atan2(S2, C2), atan2(-S2, C2)]

        solutions = []

        for q2 in q2s:
            k1 = L1 + L2 * cos(q2)
            k2 = L2 * sin(q2)
            q1 = atan2(py, px) - atan2(k2, k1)
            solutions.append((q1, q2))

        return solutions

    def mat_transfo(self, qs):
        
        c1 = cos(qs[0])
        s1 = sin(qs[0])
        c2 = cos(qs[1])
        s2 = sin(qs[1])

        T= [[c1*c2,s1, c1*s2,s1*self.l2],
        [s1*c2,-c1,s1*s2,-c1*self.l2],
        [-s2,0,-c2,self.l1],
        [0,0,0,1]]
        
        return T

    def mgd(self, q):
        return self.mat_transfo(q)[:3, 3]
    
    def jacobienne(self, q1, q2):
        
        L1, L2 = self.l1, self.l2
        
        s1 = sin(q1)
        c1 = cos(q1)
        s12 = sin(q1 + q2)
        c12 = cos(q1 + q2)
        
        # Dérivées partielles de x par rapport à q1 et q2
        dx_dq1 = -L1 * s1 - L2 * s12
        dx_dq2 = -L2 * s12
        
        # Dérivées partielles de y par rapport à q1 et q2
        dy_dq1 = L1 * c1 + L2 * c12
        dy_dq2 = L2 * c12
        
        return [[dx_dq1, dx_dq2], [dy_dq1, dy_dq2]]
    
    def zone_atteignable(self, position_cible):
        px, py = position_cible.x - self.pivot1.getPosition().x, position_cible.y - self.pivot1.getPosition().y
        distance = sqrt(px**2 + py**2)
        if distance > (self.l1 + self.l2) or distance < abs(self.l1 - self.l2):
            return False
        return True

    def mci(self, position_cible, q_init=None, tol=1e-5, max_iter=50):
        
        if self.zone_atteignable(position_cible) == False:
            return [(0,0)]
        
        if q_init is None:
            q_init = (self.barre1.theta, self.barre2.theta - self.barre1.theta)

        target = np.array([
            position_cible.x - self.pivot1.getPosition().x,
            position_cible.y - self.pivot1.getPosition().y
        ])
        
        q = np.array(q_init)
        
        L1, L2 = self.l1, self.l2

        for i in range(max_iter):
            q1, q2 = q[0], q[1]

            c1, s1 = cos(q1), sin(q1)
            c12, s12 = cos(q1 + q2), sin(q1 + q2)
            
            current_pos = np.array([
                L1 * c1 + L2 * c12,
                L1 * s1 + L2 * s12
            ])
            
            error = target - current_pos
            if np.linalg.norm(error) < tol:
                return [tuple(q.tolist())]
            
            J = np.array(self.jacobienne(q1, q2))
            
            dq = np.linalg.pinv(J) @ error
            
            q = q + dq
            
        return [tuple(q.tolist())]
        
    def gameDraw(self, screen, scale):
        self.barre1.gameDraw(screen, scale)
        self.barre2.gameDraw(screen, scale)
        self.pivot1.gameDraw(screen, scale)
        self.pivot2.gameDraw(screen, scale)

if __name__ == "__main__":

    def bras2R():
        from multiverse import Univers, InputBox
        from control_pid import ControlPID_position

        uni = Univers(name="Bras 2R Interactif", game=True, dimensions=(10, 10))
        robot = Bras2R(angle1=0, angle2=0)

        pid1 = ControlPID_position(moteur=robot.moteur1, K_P=3000, K_I=20, K_D=800,getPosition=lambda: robot.barre1.theta,getSpeed=lambda: robot.barre1.omega)
        pid1.modulo = True

        pid2 = ControlPID_position(moteur=robot.moteur2, K_P=3000, K_I=20, K_D=800,getPosition=lambda: robot.barre2.theta - robot.barre1.theta,getSpeed=lambda: robot.barre2.omega - robot.barre1.omega)
        pid2.modulo = True

        def angle_1(text):
            try:
                val = radians(float(text))
                pid1.setTarget(val)
            except: pass

        def angle_2(text):
            try:
                val = radians(float(text))
                pid2.setTarget(val)
            except: pass

        box1 = InputBox(20, 50, 100, 32, text='0', title='Angle 1 (en °) :', callback=angle_1)
        box2 = InputBox(20, 120, 100, 32, text='0', title='Angle 2 (en °) :', callback=angle_2)

        uni.addObjets(robot, pid1, pid2, box1, box2)
        uni.simulateRealTime()

    bras2R()

if __name__ == "__main__":

    def bras2R_mgi():
        
        from multiverse import Univers, v, Particule, Button
        from control_pid import ControlPID_position

        uni = Univers(name="Bras 2R Interactif", game=True, dimensions=(10, 10))
        robot = Bras2R(angle1=0, angle2=0)

        pid1 = ControlPID_position(moteur=robot.moteur1, K_P=3000, K_I=20, K_D=800,getPosition=lambda: robot.barre1.theta,getSpeed=lambda: robot.barre1.omega)
        pid2 = ControlPID_position(moteur=robot.moteur2, K_P=3000, K_I=20, K_D=800,getPosition=lambda: robot.barre2.theta - robot.barre1.theta,getSpeed=lambda: robot.barre2.omega - robot.barre1.omega)

        uni.target = v(5, 5, 0)
        visu_target = Particule(position=uni.target, color=(255, 0, 0), masse=0)

        # ajout d'un bouton pour afficher les solutions
        numero_sol = [0] 

        def action_bouton():
            numero_sol[0] += 1
            solutions = robot.mgi(uni.target)
            if solutions:
                idx = numero_sol[0] % len(solutions)
                pid1.setTarget(solutions[idx][0])
                pid2.setTarget(solutions[idx][1])

        button = Button(10, 80, 200, 32, text='Switch Solution', callback=action_bouton)
        uni.addObjets(robot, pid1, pid2, visu_target, button)

        def interaction(self, events, keys):
            # Changement de cible à la souris
            for event in events:
                if event.type == pygame.MOUSEBUTTONDOWN:
                    if button.rect.collidepoint(event.pos):
                        continue
                    mx, my = pygame.mouse.get_pos()
                    px = mx / self.scale
                    py = (self.gameDimensions[1] - my) / self.scale
                    uni.target = v(px, py, 0)
                    visu_target.position[-1] = uni.target

                    solutions = robot.mgi(uni.target)
                    idx = numero_sol[0] % len(solutions)

                    pid1.setTarget(solutions[idx][0])
                    pid2.setTarget(solutions[idx][1])

        uni.gameInteraction = MethodType(interaction, uni)
        uni.simulateRealTime()

    bras2R_mgi()

if __name__ == "__main__":
    def bras2R_mci():
    
        uni = Univers(name="Bras 2R mci", game=True, dimensions=(10, 10))
        
        robot = Bras2R(angle1=0.1, angle2=0.1) # Pas 0,0 pour éviter la singularité de départ
        
        pid1 = ControlPID_position(moteur=robot.moteur1, K_P=3000, K_I=20, K_D=800,getPosition=lambda: robot.barre1.theta,getSpeed=lambda: robot.barre1.omega)
        pid2 = ControlPID_position(moteur=robot.moteur2, K_P=3000, K_I=20, K_D=800,getPosition=lambda: robot.barre2.theta - robot.barre1.theta,getSpeed=lambda: robot.barre2.omega - robot.barre1.omega)
        pid1.modulo = True
        pid2.modulo = True

        uni.target = v(5, 5, 0)
        visu_target = Particule(position=uni.target, color=(0, 255, 0), masse=0)

        uni.addObjets(robot, pid1, pid2, visu_target)

        def interaction(self, events, keys):
            for event in events:
                if event.type == pygame.MOUSEBUTTONDOWN:
                    mx, my = pygame.mouse.get_pos()
                    px = mx / self.scale
                    py = (self.gameDimensions[1] - my) / self.scale
                    uni.target = v(px, py, 0)
                    visu_target.position[-1] = uni.target
                    
                    q1_actuel = robot.barre1.theta
                    q2_actuel = robot.barre2.theta - robot.barre1.theta
                    
                    solutions = robot.mci(uni.target, q_init=(q1_actuel, q2_actuel))
                    
                    if solutions:
                        q1_consigne, q2_consigne = solutions[0]
                        pid1.setTarget(q1_consigne)
                        pid2.setTarget(q2_consigne)

        uni.gameInteraction = MethodType(interaction, uni)
        uni.simulateRealTime()

    bras2R_mci()
