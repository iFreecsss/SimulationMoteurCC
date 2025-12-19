import matplotlib.pyplot as plt
import numpy as np

class MoteurCC:
    def __init__(self, resistance=1, inductance=0, fcem=0.01, couple=0.01, inertie=0.01, visc=0.1, name="MoteurCC", color=None):

        # param généraux
        self.name = name
        self.color = color

        # paramètres électriques et mécaniques du moteur
        self.R = resistance
        self.L = inductance
        self.Ke = fcem
        self.Kc = couple
        self.I = inertie
        self.Nu = visc

        # états du moteur
        self.Um = 0
        self.i = 0
        self.omega = 0
        self.theta = 0

        # Liste pour l'historique
        self.time_history = []
        self.speed_history = []
        self.temps = 0

        # ajout des perturbations externes
        self.J_charge = 0.0
        self.f_charge = 0.0
        self.C_ext = 0.0 


    def __str__(self):return (f"MoteurCC(R={self.R}, L={self.L}, Ke={self.Ke}, Kc={self.Kc}, I={self.I}, Nu={self.Nu})")
    def __repr__(self):return str(self)
    
    def setCharge(self, inertie_charge, frottement_charge=0):
        self.J_charge = inertie_charge
        self.f_charge = frottement_charge

    def setCoupleExterne(self, couple_resistant):self.C_ext = couple_resistant
    def setVoltage(self, V):self.Um = V
    
    def getPosition(self): return self.theta
    def getSpeed(self): return self.omega
    def getTorque(self): return self.couple
    def getIntensity(self): return self.i
    
    

    def simule(self, step):
        """
        Simule le moteur pendant un pas de temps 'step'.

        Parameters:
            step (float): Pas de temps de la simulation en secondes.

        Returns:
            None: Met à jour les états internes du moteur.
        """
            
        # on calcule la force électromotrice
        E = self.Ke * self.omega

        # si l'inductance est nulle, on est en régime statique
        # sinon on est en régime dynamique
        if self.L < 1e-9: 
            self.i = (self.Um - E) / self.R
        else:
            # di/dt = (Um - E - R*i) / L
            di_dt = (self.Um - E - self.R * self.i) / self.L
            self.i += di_dt * step # intégration

        # calcule du couple utile qui est simplement la différence 
        # entre le couple moteur et le couple résistant extérieur
        self.couple = self.Kc * self.i - self.C_ext

        J_tot = self.I + self.J_charge
        f_tot = self.Nu + self.f_charge
        
        # dOmega/dt = (Couple - f * Omega) / J
        domega_dt = (self.couple - f_tot * self.omega) / J_tot

        # intégration
        self.omega += domega_dt * step
        self.theta += self.omega * step
        
        # sauvegarde pour l'affichage
        self.temps += step
        self.time_history.append(self.temps)
        self.speed_history.append(self.omega)

    def plot(self, modele_theorique=False):
        if self.color is None:
            # renvoie un tableau avec 3 valeurs aléatoires entre 0 et 1 pour définir aleatoirement une couleur
            self.color = np.random.rand(3,)

        t = np.array(self.time_history)
        vitesses = np.array(self.speed_history)

        plt.plot(t, vitesses, color=self.color, label=self.name, linewidth=2)

        # pour éviter de tracer la courbe théorique dès qu'on plot
        if modele_theorique:
            # courbe issue de la théorie avec L=0
            # Omega(t) = K * U * (1 - exp(-t/tau))
            deno = self.R * self.Nu + self.Kc * self.Ke
            
            if deno != 0:
                K = self.Kc / deno
                tau = (self.R * self.I) / deno
                
                vitesse_theo = K * self.Um * (1 - np.exp(-t / tau))

                plt.plot(t, vitesse_theo, 'r--', label='Modèle analytique', linewidth=2)
        
if __name__ == "__main__":

    plt.figure(figsize=(30, 20))

    # moteur 1 sans charge
    m1 = MoteurCC(name="Moteur sans charge")
    m1.setVoltage(1)

    # moteur 2 avec une inertie de charge et des frottements supplémentaires
    m2 = MoteurCC(name="Moteur avec charge")
    m2.setVoltage(1)
    m2.setCharge(inertie_charge=0.05)
    
    # moteur 3 avec un couple extérieur résistant
    m3 = MoteurCC(name="Moteur avec couple extérieur")
    m3.setVoltage(1)
    m3.setCoupleExterne(couple_resistant=0.0005)

    # moteur 4 avec sans inertie de charge mais avec des frottements supplémentaires
    m4 = MoteurCC(name="Moteur avec frottements supplémentaires")
    m4.setVoltage(1)
    m4.setCharge(inertie_charge=0, frottement_charge=0.02)

    t = 0
    step = 0.01
    duration = 5.0
    
    while t < duration:
        m1.simule(step)
        m2.simule(step)
        m3.simule(step)
        m4.simule(step)
        t += step

    m1.plot(True)
    m2.plot()
    m3.plot()
    m4.plot()
    
    plt.title("Réponse indicielle en vitesse de différents moteurs à courant continu")
    plt.xlabel("Temps (s)")
    plt.ylabel("Vitesse (rad/s)")
    plt.grid()
    plt.legend()
    plt.show()




"""Classe ControlPID_vitesse :
attributs :
• paramètres K_P,K_I,K_D, moteurCC à contrôler
• Entrée : Vitesse désirée
• Sortie : tension envoyée au moteur ; vitesse du moteur
Méthodes
• _init_, _str_, _repr_
• setTarget(vitesse)
• getVoltage()
• simule(step) [ avec m.setVoltage(V) et m.simule(step)]
• plot()"""