from cmath import cos
from math import atan2, sin
import math


class Vector2d:
    def __init__(self, x: float = 0, y: float = 0, angle: float = 0, magnitude: float = 0):
        self.x = x
        self.y = y

        self.angle = angle
        self.magnitude = magnitude

        if self.x != 0 and self.y != 0:
            self.angle = atan2(self.y, self.x) * 180 / 3.141592653589793
            self.magnitude = (self.x ** 2 + self.y ** 2) ** 0.5
        if self.x == 0 and self.y == 0:
            self.x = self.magnitude * cos(math.radians(self.angle))
            self.y = self.magnitude * sin(math.radians(self.angle))

    def getPoint(self):
        return (self.x, self.y)
  
    def addTuple(self,tuple):
        point = (self.x + tuple[0], self.y + tuple[1])
        return point