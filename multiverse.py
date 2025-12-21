from control_pid import ControlPID_vitesse
from moteur_cc import MoteurCC
from vecteur3d import Vector3D as v
from types import MethodType
import pygame

class Univers:
    def __init__(self, name='ici', t0=0, step=0.001, dimensions=(100,100), game=False, gameDimensions=(1024,780), fps=60):
        self.name = name
        self.time = [t0]
        self.population = []
        self.generators = []
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
        
    def addMotor(self, *members):
        for i in members:
            self.population.append(i)
            
    def simulateAll(self):
        for p in self.population:
            p.simule(self.step)
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

            # Gestion des interactions utilisateur (clavier/souris)
            self.gameInteraction(events, keys) 
            
            # Simulation physique (1 pas graphique = 1/FPS secondes de physique)
            self.simulateFor(1 / self.gameFPS)    
            
            # Dessin de tous les objets
            for t in self.population:
                t.gameDraw(screen, self.scale)

            flip_surface = pygame.transform.flip(screen, False, flip_y=True)
            screen.blit(flip_surface, (0, 0))
            
            # Affichage du temps (Il faut le redessiner après le flip pour qu'il soit à l'endroit)
            # Note: Si on flip tout l'écran, le texte sera à l'envers. 
            # Astuce : On blit le texte APRES le flip ou on ne flip pas le texte.
            # Ici, je simplifie : on affiche le texte en haut à gauche "proprement".
            
            # On ré-affiche le texte sur l'écran final (non flippé) pour qu'il soit lisible
            text_surface_obj = font_obj.render(('Time: %.2f s' % self.time[-1]), True, (0,0,0))
            screen.blit(text_surface_obj, (10, 10))
            
            pygame.display.flip()
            clock.tick(self.gameFPS)
        
        pygame.quit()



if __name__ == "__main__":
    u = Univers(name='Simulation Moteur', game=True)
    
    m1 = MoteurCC(name='Moteur A', position=v(5, 5, 0))
    
    u.addMotor(m1)
    
    def interaction_clavier(self, events, keys):
        moteur = self.population[0]
        
        if keys[pygame.K_UP]:
            moteur.setVoltage(12.0)
        elif keys[pygame.K_DOWN]:
            moteur.setVoltage(-12.0)
        elif keys[pygame.K_SPACE]:
             moteur.setVoltage(0)
        else:
            moteur.setVoltage(0) # On relâche
            
    # On attache la fonction à l'univers
    u.gameInteraction = MethodType(interaction_clavier, u)

    
    u.simulateRealTime()