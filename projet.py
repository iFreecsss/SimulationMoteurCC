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
        pass


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