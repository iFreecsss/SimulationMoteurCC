import matplotlib.pyplot as plt
import numpy as np

class MoteurCC:
    def __init__(self, resistance=1, inductance=0, fcem=0.01, couple=0.01, inertie=0.01, visc=0.1):

        self.R = resistance
        self.L = inductance
        self.Ke = fcem
        self.Kc = couple
        self.I = inertie
        self.Nu = visc

        # Variables d'état
        self.Um = 0
        self.i = 0
        self.omega = 0
        self.theta = 0

        # Liste pour l'historique (pour plot)
        self.time_history = []
        self.speed_history = []
        self.current_time = 0


    def __str__(self):        return (f"MoteurCC(R={self.R}, L={self.L}, Ke={self.Ke}, "
                f"Kt={self.Kt}, I={self.I}, Nu={self.Nu})")
    def __repr__(self):
        return str(self)
    

    def setVoltage(self, V):
        self.V = V
    def getPosition(self):
        return self.position
    def getSpeed(self):
        return self.speed
    def getTorque(self):
        return self.torque
    def getIntensity(self):
        return self.intensity
    

    def simule(self, step):
        """
        Simule un pas de temps (méthode d'Euler)[cite: 55].
        Intègre les équations différentielles électrique et mécanique.
        """
        # 1. Calcul de la force contre-électromotrice E(t) = ke * Omega(t) [cite: 22]
        E = self.Ke * self.omega
        
        # 2. Équation Électrique : Um = E + Ri + L(di/dt) [cite: 25]
        # Si L est très proche de 0, on utilise la relation algébrique (L=0) 
        if self.L < 1e-6: 
            self.i = (self.Um - E) / self.R
        else:
            di_dt = (self.Um - E - self.R * self.i) / self.L
            self.i += di_dt * step # Intégration d'Euler

        # 3. Calcul du Couple Moteur : Gamma = kc * i [cite: 16]
        self.couple = self.Kc * self.i
        
        # 4. Équation Mécanique : J(dOmega/dt) + f*Omega = Gamma [cite: 27]
        # Couple résistant (ici seulement frottement f*Omega, pas de charge externe par défaut)
        domega_dt = (self.couple - self.Nu * self.omega) / self.I

        # 5. Mise à jour des variables d'état (Intégration)
        self.omega += domega_dt * step
        self.theta += self.omega * step
        
        # 6. Mise à jour de l'historique pour plot()
        self.current_time += step
        self.time_history.append(self.current_time)
        self.speed_history.append(self.omega)

    def plot(self):
        plt.plot(self.time_history, self.speed_history, label='Vitesse $\Omega(t)$')
        plt.xlabel('Temps (s)')
        plt.ylabel('Vitesse (rad/s)')
        plt.title('Réponse du Moteur CC')
        plt.grid()
        plt.legend()

if __name__ == "__main__":
    m = MoteurCC()
    t = 0
    step = 0.01
    temps = [t]
    while t<2 :
        t=t+step
        temps.append(t)
        m.setVoltage(1)
        m.simule(step)
        m.plot()



"""mécaniques
• Entrées : Tension, charge (inertie ajoutée), couple extérieur
résistif, viscosité
• Sorties : Courant, couple, vitesse, position
méthodes :
• _init_, _str_, _repr_
• setVoltage(V)
• getPosition(), getSpeed(), getTorque(), getIntensity()
• simule(step)
• plot()"""