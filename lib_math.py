# -*- coding: utf-8 -*-
"""
Librerpia matemática 

Universidad del valle de Guatemala 


"""

from array import array

class Matrix(object):
    # Matrix multiplication AB
    def __init__(self, matrix: array):
        self.matrix = matrix   

    def matrix(self, B):
        N = len(self.matrix)
        M = len(self.matrix[0])
        if type(B.matrix[0]) is list:
            P = len(B.matrix[0])
        else:
            P = 1
        
        result = []
        
        for i in range(N):
            result.append([])
            for j in range(P):
                result[i].append(0)
        if type(B.matrix[0]) is list:
            for i in range(N):
                for j in range(P):
                    for k in range(M):
                        result[i][j] += self.matrix[i][k] * B.matrix[k][j]
        else:
            for i in range(N):
                for j in range(P):
                    for k in range(M):
                        result[i][j] += self.matrix[i][k] * B.matrix[k]
        return Matrix(result)
    
    
class V3(object):
    def __init__(self, x, y = 0, z=0, w=1):
        if (type(x) == Matrix):
            self.x =x.matrix[0][0]
            self.y =x.matrix[1][0]
            self.z =x.matrix[2][0]
            self.w =x.matrix[3][0]
        else:
            self.x = x
            self.y = y
            self.z = z
            self.w = w

    def round(self):
        self.x = round(self.x)
        self.y = round(self.y)
        self.z = round(self.z)

    def summ(self,other):
        return V3(
            self.x + other.x,
            self.y + other.y,
            self.z + other.z
        )

    def subs(self,other):
        return V3(
            self.x - other.x,
            self.y - other.y,
            self.z - other.z
        )

    def mult(self,other):
        if (type(other) == int) or (type(other) == float):
            return V3(
                self.x * other,
                self.y * other,
                self.z * other
            )
        return V3(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x
        )

    def matrix(self,other):
        return self.x * other.x + self.y * other.y + self.z * other.z

    def length(self):
        return (self.x**2+self.y**2+self.z**2)**0.5

    def norm(self):
        return self * (1/self.__length__())

    def rep(self):
        return "V3(%s,%s,%s)" % (self.x,self.y,self.z)
