from moteur_cc import MoteurCC

class ControlPID_vitesse:
    def __init__(self,moteur, K_P, K_I, K_D):

        self.K_P = K_P
        self.K_I = K_I
        self.K_D = K_D

        self.moteur = moteur

        self.vitesse_desiree = 0
        self.erreur_precedente = 0
        self.integrale = 0


    def __str__(self):
        return f"ControlPID(K_P={self.K_P}, K_I={self.K_I}, K_D={self.K_D}, moteur={self.moteur})"
    def __repr__(self):
        return str(self)

    def setTarget(self, vitesse):
        self.vitesse_desiree = vitesse

    def getStaticError(self):
        return self.vitesse_desiree - self.moteur.getSpeed()
    
    def getTheoricalStaticError(self):
        # si on a un terme intégral, l'erreur statique est nulle théoriquement
        if self.K_I > 0:
            return 0.0

        # pour un P ou un PD on a erreur = consigne / (1 + GAIN_MOTEUR * K_P)        
        K_moteur = self.moteur.Kc / ((self.moteur.R * self.moteur.Nu) + (self.moteur.Kc * self.moteur.Ke))
        
        K_BO = K_moteur * self.K_P
        
        return self.vitesse_desiree / (1 + K_BO)
    
    def getTimeResponse(self, pourcent=5):

        ts = self.moteur.time_history
        vs = self.moteur.speed_history

        seuil = self.vitesse_desiree * (1 - pourcent / 100)

        for t, v in zip(ts, vs):
            if v > seuil:
                return t
        return f"Temps de réponse à {pourcent}% non atteint."


    def simule(self, step):

        vitesse_actuelle = self.moteur.getSpeed()
        erreur = self.vitesse_desiree - vitesse_actuelle

        self.integrale += erreur * step
        derivee = (erreur - self.erreur_precedente) / step if step > 0 else 0

        # K_P : si je suis loin de la cible alors j'augmente la tension
        # K_I : si ça fait longtemps qu'on a une erreur alors j'augmente la tension
        # K_D : si je me rapproche vite de la cible alors je diminue la tension pour éviter le dépassement
        tension = (self.K_P * erreur) + (self.K_I * self.integrale) + (self.K_D * derivee)

        self.moteur.setVoltage(tension)
        self.moteur.simule(step)

        self.erreur_precedente = erreur

    def plot(self, moteur_compare=None):
        import matplotlib.pyplot as plt
        import numpy as np

        plt.figure(figsize=(30, 20))

        self.moteur.plot()
        if moteur_compare is not None:
            for moteur in moteur_compare:
                moteur.plot()
        plt.legend()

    def plotVoltage(self):
        import matplotlib.pyplot as plt
        import numpy as np

        color = np.random.rand(3,)

        t = np.array(self.moteur.time_history)
        tensions = np.array(self.moteur.voltage_history)

        plt.plot(t, tensions, color=color, label=f'Tension appliquée à {self.moteur.name}')
        plt.legend()


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    m_bo = MoteurCC(name="Moteur BO")
    m_bf_PID = MoteurCC(name="Moteur BF PID")
    m_bf_PI = MoteurCC(name="Moteur BF PI")
    m_bf_P = MoteurCC(name="Moteur BF P")

    control_PID = ControlPID_vitesse(m_bf_PID, K_P=20, K_I=50, K_D=0.1)
    control_PI = ControlPID_vitesse(m_bf_PI, K_P=20, K_I=100, K_D=0.0)
    control_P = ControlPID_vitesse(m_bf_P, K_P=20, K_I=0.0, K_D=0.0)

    t = 0
    step = 0.001
    duration = 3.0

    target = 2 # rad/s
    K = m_bo.Kc / (m_bo.R * m_bo.Nu + m_bo.Kc * m_bo.Ke)

    while t < duration:

        m_bo.setVoltage(target/K)
        control_PID.setTarget(target)
        control_PI.setTarget(target)
        control_P.setTarget(target)

        control_PID.simule(step)
        control_PI.simule(step)
        control_P.simule(step)
        m_bo.simule(step)

        t += step

    control_PID.plot([m_bo, m_bf_PI, m_bf_P])
    plt.show()

    plt.figure(figsize=(30, 20))
    control_PID.plotVoltage()
    control_PI.plotVoltage()
    control_P.plotVoltage()
    plt.show()

    print("Temps de réponse PID :", control_PID.getTimeResponse())
    print("Temps de réponse PI :", control_PI.getTimeResponse())
    print("Temps de réponse P :", control_P.getTimeResponse())

    # pas de dépassement pour le PI car on a de une viscosité relativement élevée par rapport à l'inertie.
    # Par contre en augmentant K_I je peux en faire apparaître.
    # erreur stat theo = erreur stat simu

    print("Erreur statique théorique PID :", control_PID.getTheoricalStaticError())
    print("Erreur statique simulée PID :", control_PID.getStaticError())