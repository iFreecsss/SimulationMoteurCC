from control_pid import ControlPID_vitesse
from moteur_cc import MoteurCC
from vecteur3d import Vector3D as v
from types import MethodType
import pygame
from pygame.locals import *

class Univers:
    def __init__(self, name='ici', t0=0, step=0.001, dimensions=(100,100), game=False, gameDimensions=(1024,780), fps=60):
        self.name = name
        self.time = [t0]
        self.population = []
        self.moteurs = []
        self.controleurs = []
        self.generators = []
        self.objects = []
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
        
    def addMotors(self, *motors):
        for m in motors:
            self.moteurs.append(m)

    def addControleurs(self, *controleurs):
        for c in controleurs:
            self.controleurs.append(c)

    def addParticules(self, *particles):
        for p in particles:
            self.population.append(p)

    def addObjects(self, *objects):
        for o in objects:
            self.objects.append(o)

    def addGenerators(self, *generators):
        for g in generators:
            self.generators.append(g)

    def simulateAll(self):
        for p in self.population:
            for source in self.generators:
                source.setForce(p)
            p.simule(self.step)

        for o in self.objects:
            for source in self.generators:
                source.setForce(o)
            o.simule(self.step)


        moteurs_controlles = set()
            
        for c in self.controleurs:
            c.simule(self.step)
            moteurs_controlles.add(c.moteur)

        for m in self.moteurs:
            if m not in moteurs_controlles:
                m.simule(self.step)
        
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
            
            # Fond d'écran (Gris clair)
            screen.fill((240, 240, 240))
            
            # Gestion des événements Pygame
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

            for m in self.moteurs:
                m.gameDraw(screen, self.scale)
            for p in self.population:
                p.gameDraw(screen, self.scale)
            for o in self.objects:
                o.gameDraw(screen, self.scale)
            for g in self.generators:
                if hasattr(g, 'drawArea'):
                    g.drawArea(screen, self.scale)
            
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

    u.addMotors(m1, m2)
    u.addControleurs(m2_PID)

    def interaction_clavier(self, events, keys):
        moteur = self.moteurs[0]

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