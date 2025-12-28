
class Torseur:
    from vecteur3d import Vector3D as v

    def __init__(self, P=v(), R=v(), M=v()):
    
        self.P = P
        self.R = R
        self.M = M

    def __str__(self):
        return f"Torseur(\n  Point: {self.P},\n  R: {self.R},\n  M: {self.M}\n)"
    
    def __repr__(self):
        return str(self)
    
    def changePoint(self, P_new=v()):
        # Résultantes identiques mais moment changent avec BABAR : M_A + \vec{BA} \wedge R
        M_new = self.M + (self.P - P_new) * self.R
        self.P = P_new
        self.M = M_new

    def __add__(self, other):
        if other.P != self.P:
            o = other.P # sauvegarde pour le remettre à ce point après
            other.changePoint(self.P)
            T = Torseur(self.P, self.R + other.R, self.M + other.M)
            other.changePoint(o)
        else : 
            T = Torseur(self.P, self.R + other.R, self.M + other.M)
        return T

    def __neg__(self):
        return Torseur(self.P, -self.R, -self.M)

    def __sub__(self,other):
        return self + (-other)

    def __eq__(self,other):
        if other.P != self.P:
            o = other.P # sauvegarde
            other.changePoint(self.P)
            TF =  self.R == other.R and self.M == other.M
            other.changePoint(o)
        else :
            TF =  self.R == other.R and self.M == other.M
        return TF

    
if __name__ == "__main__": 
    from vecteur3d import Vector3D as v

    P = v()
    R = v(1,0,0)
    M = v(0,1,0)

    T = Torseur(P,R,M)
    T2 = Torseur(M,P,R)

    print(T,'\n+\n',T2,'\n=\n',T+T2)

    T.changePoint(v(1,1,0))

    print('T au point (1,1,0)=',T)


