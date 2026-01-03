from control_pid import ControlPID_position, ControlPID_vitesse
from moteur_cc import MoteurCC

from vecteur3d import Vector3D as v

from types import MethodType
import pygame
from pygame.locals import *

from particule import *
from forces_liaisons import *
from assemblages import *


class Univers:
    def __init__(self, name='ici', t0=0, step=0.001, dimensions=(100,100), game=False, gameDimensions=(1024,780), fps=60):
        self.name = name
        self.time = [t0]
        self.objets = {'particules': [],
                       'moteurs': [],
                       'controleurs': [],
                       'generators': [],
                       'liaisons': [],
                       'constraints': [],
                       'objets': []}

        self.step = step
        
        self.dimensions = dimensions
        
        self.game = game
        self.gameDimensions = gameDimensions
        self.gameFPS = fps
        
        self.scale = gameDimensions[0] / dimensions[0]
        
    def __str__(self):
        return 'Univers (%s, time=%.2f, step=%g)' % (self.name, self.time[-1], self.step)
        
    def __repr__(self):
        return str(self)
        
    def addObjets(self, *objets):
        for o in objets:
            if isinstance(o, Particule):
                self.objets['particules'].append(o)
            elif isinstance(o, MoteurCC):
                self.objets['moteurs'].append(o)
            elif isinstance(o, (ControlPID_vitesse, ControlPID_position)):
                self.objets['controleurs'].append(o)
            elif isinstance(o, Force):
                self.objets['generators'].append(o)
            elif isinstance(o, (Pivot, PivotMobile)): 
                self.objets['constraints'].append(o)
            elif isinstance(o, (Gravity, Force, ForceSelect, Viscosity, SpringDamper, Fil, Glissiere)):
                self.objets['liaisons'].append(o)
            elif isinstance(o, BrasCentrifuge):
                self.objets['generators'].append(o)
            elif isinstance(o, (Barre2D, Pendule, TurtleBot, PenduleBarre2D)):
                self.objets['objets'].append(o)
            else:
                if hasattr(o, 'gameDraw'):
                     self.objets['objets'].append(o)
                print(f"Note : Objet '{type(o).__name__}' ajouté de manière générique.")

    def simulateAll(self):
        # on sépare les objets en fonction de leur role 
        receveurs = self.objets['particules'] + self.objets['objets'] + self.objets['moteurs']
        sources = self.objets['generators'] + self.objets['liaisons']

        for source in sources:
            for rec in receveurs:
                if hasattr(source, 'setForce'):
                    source.setForce(rec)

        moteurs_controles = set()

        for c in self.objets['controleurs']:
            c.simule(self.step)
            moteurs_controles.add(c.moteur)

        objets_contraints = set()

        for constraint in self.objets['constraints']:
            if hasattr(constraint, 'simule'):
                constraint.simule(self.step)

        for rec in receveurs:
            if rec in moteurs_controles:
                continue

            if rec in objets_contraints:
                continue
            
            if hasattr(rec, 'simule'):
                rec.simule(self.step)

        self.time.append(self.time[-1] + self.step)


    def simulateFor(self, duration):
        while duration > 0:
            self.simulateAll()
            duration -= self.step
       
    def gameInteraction(self, events, keys):
        # Fonction qui sera surchargée par le client
        pass
    
    def simulateRealTime(self):
        pygame.init()
        W, H = self.gameDimensions
        screen = pygame.display.set_mode((W, H))   
        clock = pygame.time.Clock()
        
        font_obj = pygame.font.SysFont('Arial', 22)

        running = True

        while running:
            
            screen.fill((240, 240, 240))
            
            pygame.event.pump()
            keys = pygame.key.get_pressed()
            events = pygame.event.get()
            
            if keys[pygame.K_ESCAPE]:
                running = False
                
            for event in events:
                if event.type == pygame.QUIT:
                    running = False

            self.gameInteraction(events, keys) 
            
            self.simulateFor(1 / self.gameFPS)    

            for item in self.objets.values():
                for obj in item:
                    if hasattr(obj, 'gameDraw'):
                        obj.gameDraw(screen, self.scale)


            # get y axis upwards, origin on bottom left : La fenetre pygame a l'axe y vers le bas. On le retourne.
            flip_surface = pygame.transform.flip(screen, False, flip_y=True)
            screen.blit(flip_surface, (0, 0))
            # On ré-affiche le texte sur l'écran final (non flippé) pour qu'il soit lisible
            text_surface_obj = font_obj.render(('Time: %.2f s' % self.time[-1]), True, (0,0,0))
            screen.blit(text_surface_obj, (10, 10))
            
            
            pygame.display.flip()  # envoie de la fenetre vers l'écran
            clock.tick(self.gameFPS)
        
        pygame.quit()



if __name__ == "__main__":
    import matplotlib.pyplot as plt
    u = Univers(name='Simulation Moteur', game=True)
    
    m1 = MoteurCC(visc=1e-3, name='Moteur BO', position=v(50, 50, 0))
    m2 = MoteurCC(visc=1e-3, name='Moteur BF', position=v(50, 30, 0))
    m2_PID = ControlPID_vitesse(m2, K_P=50.0, K_I=10, K_D=0.1)

    u.addObjets(m1, m2)
    u.addObjets(m2_PID)

    def interaction_clavier(self, events, keys):
        moteur = self.objets['moteurs'][0]

        if keys[pygame.K_UP]:
            moteur.setVoltage(12.0)
        elif keys[pygame.K_DOWN]:
            moteur.setVoltage(-12.0)
        elif keys[pygame.K_SPACE]:
            moteur.setVoltage(0)

    u.gameInteraction = MethodType(interaction_clavier, u)

    m2_PID.setTarget(10)
    u.simulateRealTime()
    
    m2_PID.plot([m1])
    plt.show()







