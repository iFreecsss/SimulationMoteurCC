class ControlPID_vitesse:
    def __init__(self,moteur, K_P, K_I, K_D, in_univers=True):

        self.K_P = K_P
        self.K_I = K_I
        self.K_D = K_D

        self.moteur = moteur

        self.vitesse_desiree = 0
        self.erreur_prec = 0
        self.int = 0

        # Flag pour indiquer si le contrôleur est dans l'univers de simulation 
        # permet de ne pas devoir simuler d'un cote les moteurs en plus des controleur 
        # en revanche dans l'univers on ne veut pas ça sinon les moteurs seraient simulés deux fois
        self.in_univers = in_univers


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
                return f"Temps de réponse à {pourcent}% : {t} s"
        return f"Temps de réponse à {pourcent}% non atteint."


    def simule(self, step):

        vitesse_actuelle = self.moteur.getSpeed()
        erreur = self.vitesse_desiree - vitesse_actuelle

        self.int += erreur * step
        derivee = (erreur - self.erreur_prec) / step if step > 0 else 0

        # K_P : si je suis loin de la cible alors j'augmente la tension
        # K_I : si ça fait longtemps qu'on a une erreur alors j'augmente la tension
        # K_D : si je me rapproche vite de la cible alors je diminue la tension pour éviter le dépassement
        tension = (self.K_P * erreur) + (self.K_I * self.int) + (self.K_D * derivee)

        self.moteur.setVoltage(tension)

        if not self.in_univers:
            self.moteur.simule(step)

        self.erreur_prec = erreur

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
    from moteur_cc import MoteurCC

    def run_sim_vitesse(target=2.0, duration=3.0, step=0.001):
        import matplotlib.pyplot as plt

        m_bo = MoteurCC(name="Moteur BO")
        m_bf_PID = MoteurCC(name="Moteur BF PID")
        m_bf_PI = MoteurCC(name="Moteur BF PI")
        m_bf_P = MoteurCC(name="Moteur BF P")

        control_PID = ControlPID_vitesse(m_bf_PID, K_P=20, K_I=50, K_D=0.1, in_univers=False)
        control_PI = ControlPID_vitesse(m_bf_PI, K_P=20, K_I=100, K_D=0.0, in_univers=False)
        control_P = ControlPID_vitesse(m_bf_P, K_P=20, K_I=0.0, K_D=0.0, in_univers=False)

        t = 0

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

    # run_sim_vitesse(target=2.0, duration=5.0, step=0.001)

class ControlPID_position:
    def __init__(self, moteur, K_P, K_I, K_D, in_univers=True, getPosition=None, getSpeed=None):
        self.moteur = moteur
        self.K_P = K_P
        self.K_I = K_I
        self.K_D = K_D

        self.position_desiree = 0.0
        self.erreur_prec = 0.0
        self.int = 0.0

        self.in_univers = in_univers

        self.getSpeed = getSpeed if getSpeed else moteur.getSpeed
        self.getPosition = getPosition if getPosition else moteur.getPosition

    def setTarget(self, angle_rad):
        self.position_desiree = angle_rad

    def simule(self, step):
        
        erreur = self.position_desiree - self.getPosition()

        # terme intégral
        self.int += erreur * step

        # terme dérivé
        # d erreur /dt = d(target - theta)/dt = 0 - d theta/dt = -vitesse angulaire
        derivee = -self.getSpeed()

        tension = (self.K_P * erreur) + (self.K_I * self.int) + (self.K_D * derivee)

        self.moteur.setVoltage(tension)

        if not self.in_univers:
            self.moteur.simule(step)

        self.erreur_prec = erreur
        
    def getStaticError(self):
        return self.position_desiree - self.moteur.getPosition()

    def plot(self, moteur_compare=None):
        import matplotlib.pyplot as plt

        self.moteur.plot_pos()
        plt.legend()

        if moteur_compare:
            for moteur in moteur_compare:
                moteur.plot_pos()
                plt.legend()


if __name__ == "__main__":
    from moteur_cc import MoteurCC
    from math import pi

    def run_sim_pos(target=1.57, duration=10.0, step=0.001):
        import matplotlib.pyplot as plt

        m_bf_PID = MoteurCC(name="Moteur BF PID")
        m_bf_PI = MoteurCC(name="Moteur BF PI")
        m_bf_P = MoteurCC(name="Moteur BF P")

        control_PID = ControlPID_position(m_bf_PID, K_P=30, K_I=50, K_D=10, in_univers=False)
        control_PI = ControlPID_position(m_bf_PI, K_P=30, K_I=50, K_D=0.0, in_univers=False)
        control_P = ControlPID_position(m_bf_P, K_P=30, K_I=0.0, K_D=0.0, in_univers=False)

        t = 0

        while t < duration:

            control_PID.setTarget(target)
            control_PI.setTarget(target)
            control_P.setTarget(target)

            control_PID.simule(step)
            control_PI.simule(step)
            control_P.simule(step)

            t += step

        plt.figure(figsize=(30, 20))
        plt.axhline(target, color='r', alpha=0.5, label='Consigne')
        control_PID.plot([m_bf_PI, m_bf_P])
        plt.title("Contrôle de position avec PID, PI et P")
        plt.xlabel("Temps (s)")
        plt.ylabel("Position (rad)")
        plt.grid()
        plt.show()

    run_sim_pos(target=pi/2)

    def run_compare_PD(target=1.57, duration=10.0, step=0.001):
        import matplotlib.pyplot as plt
        import numpy as np

        Ds = [0.1,5.0,30.0, 100.0]
        Ps = [10.0, 20.0, 30.0, 80.0, 1e4]

        plt.figure(figsize=(30, 20))
        for D in Ds:
            moteur = MoteurCC(name=f"PD D={D}")
            pid = ControlPID_position(moteur, K_P=20, K_I=0, K_D=D, in_univers=False)

            t = 0
            times = []
            positions = []

            while t < duration:
                pid.setTarget(target)
                pid.simule(step)
                times.append(t)
                positions.append(moteur.getPosition())
                t += step

            plt.plot(times, positions, label=f"D={D}")
        

        plt.axhline(target, color='r', alpha=0.5, label='Consigne')
        plt.xlabel("Temps (s)")
        plt.ylabel("Position (rad)")
        plt.grid()
        plt.legend()
        plt.show()


        plt.figure(figsize=(30, 20))
        for P in Ps:
            moteur = MoteurCC(name=f"P P={P}")
            pid = ControlPID_position(moteur, K_P=P, K_I=0, K_D=0, in_univers=False)

            t = 0
            times = []
            positions = []

            while t < duration:
                pid.setTarget(target)
                pid.simule(step)
                times.append(t)
                positions.append(moteur.getPosition())
                t += step

            plt.plot(times, positions, label=f"P={P}")


        plt.axhline(target, color='r', alpha=0.5, label='Consigne')
        plt.xlabel("Temps (s)")
        plt.ylabel("Position (rad)")
        plt.grid()
        plt.legend()
        plt.show()

    run_compare_PD(target=pi/2) 
