class Vector3D:
    """Class Vector3D(x=0, y=0, z=0) representing a 3D vector

    Attributes
    ----------
    x : float | int
        The x component of the vector
    y : float | int
        The y component of the vector
    z : float | int
        The z component of the vector

    Methods
    -------
    mod() : float
        Returns the modulus of the vector
    norm() : Vector3D
        Returns the normalized vector
    normalize() : None
        Normalizes the vector in place
    __add__(other) : Vector3D
        Adds two vectors or a vector and a scalar
    __sub__(other) : Vector3D
        Subtracts two vectors or a vector and a scalar
    __mul__(other) : Vector3D
        Calculates the cross product of 2 vectors or scales the vector by a scalar
    __pow__(other) : float
        Calculates the dot product of 2 vectors
    __neg__() : Vector3D
        Returns the opposite vector
    __eq__(other) : bool
        Checks if two vectors are equal
    __str__() : str
        Returns a string representation of the vector
    __repr__() : str
        Returns a string representation of the vector
    """
    
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

    @property
    def x(self):
        return self.__x
    @property
    def y(self):
        return self.__y
    @property
    def z(self):
        return self.__z
    
    @x.setter
    def x(self, x):
        if type(x) not in [float, int]:
            raise TypeError("x doit être un float")
        self.__x = x
    @y.setter
    def y(self, y):
        if type(y) not in [float, int]:
            raise TypeError("y doit être un float")
        self.__y = y
    @z.setter
    def z(self, z):
        if type(z) not in [float, int]:
            raise TypeError("z doit être un float")
        self.__z = z

    def __pow__(self, other):
        """
        Calculate the dot product of 2 vectors.

        Parameters
        ----------
        other : Vector3D
            The other vector to calculate the dot product with.

        Returns
        -------
        float
            The dot product of the 2 vectors.
        """
        if not isinstance(other, Vector3D):
            raise TypeError(f"{other} doit être un Vector3D mais il est un {type(other)}")
        return self.x * other.x + self.y * other.y + self.z * other.z
    
    def __rpow__(self, other):
        return self**other
    
    def __str__(self):
        return f"Vector3D({self.x}, {self.y}, {self.z})"
    
    def __eq__(self, other):
        """
        Check if two vectors are equal.
        Parameters
        ----------
        other : Vector3D
            The other vector to compare.
        Returns
        -------
        bool
            True if the vectors are equal, False otherwise.
        """
        if not isinstance(other, Vector3D) and type(other) in [int, float]:
            other = Vector3D(other, other, other)
        if self.x == other.x and self.y == other.y and self.z == other.z:
            return True
        return False
    
    def __add__(self, other):
        """
        Add two vectors or a vector and a scalar.

        Parameters
        ----------
        other : Vector3D or float or int
            The other vector to add or the scalar to add to each component.

        Returns
        -------
        Vector3D
            The result of the addition.
        """
        if type(other) not in [Vector3D, float, int]:
            raise TypeError(f"other must be a Vector3D, a float r an integer")
        if not isinstance(other, Vector3D):
            return Vector3D(self.x + other, self.y + other, self.z + other)
        return Vector3D(self.x + other.x, self.y + other.y, self.z + other.z)
    
    def __radd__(self, other):
        return self + other
    
    def __sub__(self, other):
        """
        Subtract two vectors or a vector and a scalar.

        Parameters
        ----------
        other : Vector3D or float or int
            The other vector to subtract or the scalar to subtract from each component.

        Returns
        -------
        Vector3D
            The result of the subtraction.
        """
        if type(other) not in [Vector3D, float, int]:
            raise TypeError(f"other must be a Vector3D, a float r an integer")
        if not isinstance(other, Vector3D):
            return Vector3D(self.x - other, self.y - other, self.z - other)
        return Vector3D(self.x - other.x, self.y - other.y, self.z - other.z)
    
    def __rsub__(self, other):
        return -(self - other)

    def __repr__(self):
        return f"Vector3D({self.x}, {self.y}, {self.z})"
    
    def __neg__(self):
        """
        Calculate the opposite vector.

        Returns
        -------
        Vector3D
            The opposite vector.
        """
        return Vector3D(-self.x, -self.y, -self.z)
    
    def __mul__(self, other):
        """
        Calculate the cross product of 2 vectors. If other is a scalar, multiply each component by this scalar.

        Parameters
        ----------
        other : Vector3D or float or int
            The other vector to calculate the cross product with or the scalar to multiply each component by.

        Returns
        -------
        Vector3D
            The cross product of the 2 vectors or the scaled vector.
        
        """
        if not isinstance(other, Vector3D):
            return Vector3D(other * self.x, other * self.y, other * self.z)
        return Vector3D(self.y * other.z - self.z * other.y, self.z * other.x - self.x      * other.z, self.x * other.y - self.y * other.x)
    
    def __rmul__(self, other):
        """
        Calculate the cross product of 2 vectors. If other is a scalar, multiply each component by this scalar.

        Parameters
        ----------
        other : Vector3D or float or int
            The other vector to calculate the cross product with or the scalar to multiply each component by.

        Returns 
        -------
        Vector3D
            The cross product of the 2 vectors or the scaled vector.
        """
        if not isinstance(other, Vector3D):
            return self * other
        return -(self * other)

    def norm(self):
        """
        Get your normalized vector in an other object.

        Returns
        -------
        Vector3D
            The normalized vector.
        """
        m = self.mod()
        if m != 0:
            return self * (1/m)
        else: 
            return Vector3D()

    def normalize(self):
        """Normalize your vector. Modifies the vector in place to be a unit vector."""
        self.x = self.x / self.mod()
        self.y = self.y / self.mod()
        self.z = self.z / self.mod()
    
    def mod(self):
        """
        Calculate the norm/modul of the vector.
        
        Returns
        -------
        float
            The norm/modul of the vector.
        """
        return (self**self)**.5
    
    def save(self,nom='vec.dat'):
        from pickle import dump
        file = open(nom,'wb')
        dump(self,file)
        file.close

    def load(self,nom='vec.dat'):
        from pickle import load
        file = open(nom,'rb')
        temp=load(file)
        self.x = temp.x
        self.y = temp.y
        self.z = temp.z
        file.close
    
    def rotZ(self, theta):
        from math import cos, sin
        x_new = self.x * cos(theta) - self.y * sin(theta)
        y_new = self.x * sin(theta) + self.y * cos(theta)
        return Vector3D(x_new, y_new, self.z)

if __name__ == "__main__":
    
    v0 = Vector3D()
    v1 = Vector3D(1,0,0)
    v2 = Vector3D(0,1,0)
    v3 = Vector3D(0,0,1)
    v5 = Vector3D(5,7,-5)

    print(v1*v2)
    print(v2*v1)
    print(v1*v2==v3)
    print((v1*v2) - v3 == 0)

    print(v1 * 5)
    print(5 * v1)

    m = v5.mod()
    n = v5.norm()

    print(m,n)
    print(n.mod())
    print(m*n == v5)