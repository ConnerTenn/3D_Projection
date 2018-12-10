
#from tkinter import *
import numpy
import pygame
import time
import random
from MatrixCode import *
from math import *

class Vec3:
	def __init__(self, x=0,y=0,z=0):
		self.X=x
		self.Y=y
		self.Z=z
		
	def __add__(self, other):
		return Vec4(self.X + other.X, self.Y + other.Y, self.Z + other.Z)
		
	def __sub__(self, other):
		return Vec4(self.X - other.X, self.Y - other.Y, self.Z - other.Z)
		
	def __mul__(self, other):
		return Vec4(self.X * other.X, self.Y * other.Y, self.Z * other.Z)
	
	def ToMat(self):
		return Matrix.fromList([[self.X],[self.Y],[self.Z],[1]])
		
	def __str__(self):
		return "["+str(round(self.X,3))+", "+str(round(self.Y,3))+", "+str(round(self.Z,3))+"]"
		
	def __getitem__(self, index):
		if index == 0:
			return self.X
		if index == 1:
			return self.Y
		if index == 2:
			return self.Z
	
	def Copy(self):
		return Vec3(self.X, self.Y, self.Z)

	@staticmethod
	def Dot2(vec1, vec2):
		return vec1.X * vec2.X + vec1.Y * vec2.Y
	
	@staticmethod
	def Dot3(vec1, vec2):
		return vec1.X * vec2.X + vec1.Y * vec2.Y + vec1.Z * vec2.Z


class Vec4:
	def __init__(self, x=0,y=0,z=0,w=1):
		self.X=x
		self.Y=y
		self.Z=z
		self.W=w
		
	def __add__(self, other):
		return Vec4(self.X + other.X, self.Y + other.Y, self.Z + other.Z, self.W + other.W)
		
	def __sub__(self, other):
		return Vec4(self.X - other.X, self.Y - other.Y, self.Z - other.Z, self.W - other.W)
		
	def __mul__(self, other):
		return Vec4(self.X * other.X, self.Y * other.Y, self.Z * other.Z, self.W * other.W)
		
	def __str__(self):
		return "["+str(round(self.X,3))+", "+str(round(self.Y,3))+", "+str(round(self.Z,3))+", "+str(round(self.W,3))+"]"
	
	def Copy(self):
		return Vec4(self.X, self.Y, self.Z, self.W)

	@staticmethod
	def Dot2(vec1, vec2):
		return vec1.X * vec2.X + vec1.Y * vec2.Y
	
	@staticmethod
	def Dot3(vec1, vec2):
		return vec1.X * vec2.X + vec1.Y * vec2.Y + vec1.Z * vec2.Z
	
	@staticmethod
	def Dot4(vec1, vec2):
		return vec1.X * vec2.X + vec1.Y * vec2.Y + vec1.Z * vec2.Z + vec1.W * vec2.W

class Shape:
	def __init__(self, points=[]):
		self.Points = []

		for p in points:
			self.Points.append(p.Copy())
		

		
	def __getitem__(self, index):
		return self.Points[index]

	def Clockwise(self):
		None
		
	def GenerateLines(self):
		'''self.a12 = -(self.P2.Y-self.P1.Y)
		self.b12 = (self.P2.X-self.P1.X)
		self.c12 = self.a12 * self.P1.X + self.b12 * self.P1.Y'''
		
		
	def ContainsPoint(self, point, backFaceCulling):
		None
		
	def Bounds(self):
		None

	def Rasterize(self, pixelArr, Width, Height):
		None
		


		
pygame.init()
Width = 1000
Height = 600
Screen = pygame.display.set_mode((Width, Height))
PixelArr = pygame.PixelArray(Screen)

start = time.time()
a=[]
for i in range(1920*1080): a.append(1)
print("1: " + str(time.time() - start))

start = time.time()
l=len(a); i=0
while i < l: 
	a[i] = 2; i+=1
print("2: " + str(time.time() - start))

start = time.time()
for i in range(1920*1080): a[i] = 5
print("3: " + str(time.time() - start))

start = time.time()
a=[]
a=[1 for i in range(1920*1080)]
print("4: " + str(time.time() - start))

print("Done")


Run = True
while Run:		
	for event in pygame.event.get():
		if event.type == pygame.QUIT:
				Run = False

	Screen.fill((0,0,255), [[100,100], [200,200]])
	
	pygame.display.flip()