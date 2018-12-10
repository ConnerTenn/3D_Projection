
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
	
	@staticmethod
	def Dot2(vec1, vec2):
		return vec1.X * vec2.X + vec1.Y * vec2.Y
	
	@staticmethod
	def Dot3(vec1, vec2):
		return vec1.X * vec2.X + vec1.Y * vec2.Y + vec1.Z * vec2.Z
	
	@staticmethod	
	def FromMat(mat):
		w = abs(mat[3][0])
		#if mat[3][0] < 0: w=0.000000000001
		if w == 0: w = 0.000000000001
		x = mat[0][0]/w; y = mat[1][0]/w; z = mat[2][0]/w
		return Vec3(x,y,z)

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
	
	def ToMat(self):
		return Matrix.fromList([[self.X],[self.Y],[self.Z],[self.W]])
		
	def ToNDC(self):
		return Vec4(self.X/self.W, self.Y/self.W, self.Z/self.W, 1)
		
	def __str__(self):
		return "["+str(round(self.X,3))+", "+str(round(self.Y,3))+", "+str(round(self.Z,3))+", "+str(round(self.W,3))+"]"
	
	@staticmethod
	def Dot2(vec1, vec2):
		return vec1.X * vec2.X + vec1.Y * vec2.Y
	
	@staticmethod
	def Dot3(vec1, vec2):
		return vec1.X * vec2.X + vec1.Y * vec2.Y + vec1.Z * vec2.Z
	
	@staticmethod
	def Dot4(vec1, vec2):
		return vec1.X * vec2.X + vec1.Y * vec2.Y + vec1.Z * vec2.Z + vec1.W * vec2.W
	
	@staticmethod	
	def FromMat(mat):
		return Vec4(mat[0][0],mat[1][0],mat[2][0],mat[3][0])

class Triangle:
	def __init__(self, p1=Vec3(), p2=Vec3(), p3=Vec3()):
		self.P1 = p1
		self.P2 = p2
		self.P3 = p3
		
	def Clockwise(self):
		None
		
	def Transform(self, matrix):			
		self.P1 = Vec3.FromMat(matrix * self.P1.ToMat())
		self.P2 = Vec3.FromMat(matrix * self.P2.ToMat())
		self.P3 = Vec3.FromMat(matrix * self.P3.ToMat())
		
	def GenBarry(self):
		self.v0 = self.P2 - self.P1
		self.v1 = self.P3 - self.P1
		
		self.d00 = Vec3.Dot2(self.v0, self.v0)
		self.d01 = Vec3.Dot2(self.v0, self.v1)
		self.d11 = Vec3.Dot2(self.v1, self.v1)
		
		denom = (self.d00 * self.d11 - self.d01 * self.d01)
		self.invdenom = 1 / denom if not denom == 0 else 1000000
		
	
	def GenerateLines(self):
		self.a12 = -(self.P2.Y-self.P1.Y)
		self.b12 = (self.P2.X-self.P1.X)
		self.c12 = self.a12 * self.P1.X + self.b12 * self.P1.Y
		
		self.a13 = -(self.P3.Y-self.P1.Y)
		self.b13 = (self.P3.X-self.P1.X)
		self.c13 = self.a13 * self.P1.X + self.b13 * self.P1.Y
		
		self.a23 = -(self.P3.Y-self.P2.Y)
		self.b23 = (self.P3.X-self.P2.X)
		self.c23 = self.a23 * self.P2.X + self.b23 * self.P2.Y
		
	def ContainsPoint(self, point, backFaceCulling):
		if self.a12 * point.X + self.b12 * point.Y > self.c12 and self.a13 * point.X + self.b13 * point.Y < self.c13 and self.a23 * point.X + self.b23 * point.Y > self.c23: return True
		if not backFaceCulling and self.a12 * point.X + self.b12 * point.Y < self.c12 and self.a13 * point.X + self.b13 * point.Y > self.c13 and self.a23 * point.X + self.b23 * point.Y < self.c23: return True
		return False
		
	def Bounds(self):
		bounds = [0 for i in range(4)]
		bounds[0] = self.P1.X
		bounds[1] = self.P1.X
		bounds[2] = self.P1.Y
		bounds[3] = self.P1.Y
		for i in (1,2):
			bounds[0] = self[i].X if self[i].X < bounds[0] else bounds[0]
			bounds[1] = self[i].X if self[i].X > bounds[1] else bounds[1]
			bounds[2] = self[i].Y if self[i].Y < bounds[2] else bounds[2]
			bounds[3] = self[i].Y if self[i].Y > bounds[3] else bounds[3]
		return bounds
		
	def __getitem__(self, index):
		if index == 0:
			return self.P1
		if index == 1:
			return self.P2
		if index == 2:
			return self.P3
		
class Renderer:
	@staticmethod
	def __init__(width, height):
		Renderer.Resize(width,height)
		Renderer.Glob = 0
		Renderer.BackFaceCulling = True
		
	@staticmethod
	def Resize(width, height):
		Renderer.Width = width
		Renderer.Height = height
		Renderer.PixelBuff = [[[0 for i in range(3)] for y in range(Renderer.Height)] for x in range(Renderer.Width)]
		Renderer.DepthBuff = [[-1 for y in range(Renderer.Height)] for x in range(Renderer.Width)]
		
		#Renderer.Image = PhotoImage(width=Renderer.Width, height=Renderer.Width)
	
	@staticmethod
	def GetProjMat():
		n=1; f=1000; r=3; t=3
		return Matrix.fromList(                                 \
			[                                                   \
				[ n/r,   0,     0,             0             ], \
				[ 0,     n/t,   0,             0             ], \
				[ 0,     0,    -(f+n)/(f-n),  -(2*f*n)/(f-n) ], \
				[ 0,     0,    -1,             0             ]  \
			])
	
	@staticmethod
	def GetTransMat(x, y, z):
		return Matrix.fromList(
			[                          \
				[  1,   0,   0,   x ], \
				[  0,   1,   0,   y ], \
				[  0,   0,   1,   z ], \
				[  0,   0,   0,   1 ]  \
			])
			
	@staticmethod
	def GetScaleMat(x, y, z):
		return Matrix.fromList(
			[                          \
				[  x,   0,   0,   0 ], \
				[  0,   y,   0,   0 ], \
				[  0,   0,   z,   0 ], \
				[  0,   0,   0,   1 ]  \
			])

	@staticmethod
	def GetRotMatX(ang):
		return Matrix.fromList(                      \
			[                                        \
				[  1,   0,          0,          0 ], \
				[  0,   cos(ang),  -sin(ang),   0 ], \
				[  0,   sin(ang),   cos(ang),   0 ], \
				[  0,   0,           0      ,   1 ]  \
			])

	@staticmethod
	def GetRotMatY(ang):
		return Matrix.fromList(                      \
			[                                        \
				[  cos(ang),   0,   sin(ang),   0 ], \
				[  0,          1,   0,          0 ], \
				[ -sin(ang),   0,   cos(ang),   0 ], \
				[  0,          0,   0,          1 ]  \
			])

	@staticmethod
	def GetRotnMatZ(ang):
		return Matrix.fromList(                      \
			[                                        \
				[  cos(ang),  -sin(ang),   0,   0 ], \
				[  sin(ang),   cos(ang),   0,   0 ], \
				[  0,          0,          1,   0 ], \
				[  0,          0,          0,   1 ]  \
			])
	
	@staticmethod
	def Interp(p, tri, val):
		v2 = p - tri.P1
		
		d20 = Vec3.Dot2(v2, tri.v0); d21 = Vec3.Dot2(v2, tri.v1)
		
		v = (tri.d11 * d20 - tri.d01 * d21) * tri.invdenom; w = (tri.d00 * d21 - tri.d01 * d20) * tri.invdenom; u = 1 - (v + w)
		
		return val[0] * u + val[1] * v + val[2] * w
	
	@staticmethod
	def Rasterize(triangle, data): #triangle has already been converted to screen coords
		#clipping
		bounds = triangle.Bounds()
		left = max(floor(bounds[0]-1), 0); right = min(ceil(bounds[1]+1), Renderer.Width); bottom = min(ceil(bounds[3]+1), Renderer.Height); top = max(floor(bounds[2]-1), 0)
		
		triangle.GenBarry(); triangle.GenerateLines(); depth = (triangle.P1.Z, triangle.P2.Z, triangle.P3.Z)
		dat = Vec3()

		c1 = (data.P1.X, data.P2.X, data.P3.X); c2 = (data.P1.Y, data.P2.Y, data.P3.Y);	c3 = (data.P1.Z, data.P2.Z, data.P3.Z)
		for point in (Vec3(x,y) for y in range(top, bottom) for x in range(left, right)):
			if triangle.ContainsPoint(point, Renderer.BackFaceCulling):
				point.Z = Renderer.Interp(point, triangle, depth); dat.X = Renderer.Interp(point, triangle, c1); dat.Y = Renderer.Interp(point, triangle, c2); dat.Z = Renderer.Interp(point, triangle, c3)
				if point.Z > -1 and point.Z < 1: Renderer.PixelShader(point, dat)
		
	@staticmethod
	def PixelShader(point, dat):
		if Renderer.DepthBuff[point.X][point.Y] == -1 or point.Z < Renderer.DepthBuff[point.X][point.Y]:
			Renderer.DepthBuff[point.X][point.Y] = point.Z
			r = (-point.Z*point.Z*point.Z*point.Z+1); colour = [r*dat.X + 255*(1-r), r*dat.Y + 255*(1-r), r*dat.Z + 255*(1-r)]
			for i in range(3): colour[i] = min(max(round(colour[i]/(255/5))*(255/5), 0), 255)
			Renderer.PixelBuff[point.X][point.Y] = colour
	
	@staticmethod
	def Clear():
		for y in range(Renderer.Height): 
			for x in range(Renderer.Width): Renderer.PixelBuff[x][y] = [0,0,0]; Renderer.DepthBuff[x][y] = -1
	
	@staticmethod
	def Draw(surface, width, height):
		
		#array = pygame.surfarray.pixels2d(surface)
		pxarray = pygame.PixelArray(surface); buff = Renderer.PixelBuff
		
		w = width/Renderer.Width; h = height/Renderer.Height
		for y in range(Renderer.Height):
			for x in range(Renderer.Width): pxarray[x, y] = pygame.Color(int(buff[x][y][0]), int(buff[x][y][1]), int(buff[x][y][2]))
				
		#print("Draw " + str(Renderer.Glob))
		#Renderer.Glob += 1
		#canvas.create_image((0, 0), image=Renderer.Image, anchor="nw")
				
				
	@staticmethod
	def Render(triangle, matrix, data=Triangle()):
		
		#transform to projection space
		triangle.Transform(matrix)
		
		#transform to screen space
		#must preserve depth value
		screenTrans = Renderer.GetScaleMat(Renderer.Width, Renderer.Height,1) * Renderer.GetTransMat(0.5,0.5,0) * Renderer.GetScaleMat(1,-1,1)
		#screenTrans = Renderer.GetTransMat(0.5,0.5,0)
		
		
		triangle.Transform(screenTrans)
		
		Renderer.Rasterize(triangle, data)
		

class Screen:
	def __init__(self, width=1000, height=600, title="Projection"):
		self.Width=width
		self.Height=height
		self.Title=title

		self.Play=True

		'''self.Tk=Tk()
		self.Tk.title(self.Title)
		self.Tk.resizable(0,0)
		self.Canvas=Canvas(self.Tk, width=self.Width, height=self.Height)
		self.Canvas.pack()'''
		pygame.init()
		self.Screen = pygame.display.set_mode((self.Width, self.Height))
		
		#self.Tk.protocol("WM_DELETE_WINDOW", self.Close)
		
		
		self.Camera = Vec3(0,-5,-10)
		self.Rot = 0
		self.Rot2 = 0
				
		Renderer(floor(self.Width/4), floor(self.Height/4))
		Renderer.BackFaceCulling = False
		
		
		self.Surface = pygame.Surface((Renderer.Width, Renderer.Height), depth=32)

		self.Play = True
		self.Update()
		#self.Tk.mainloop()
	
	def Close(self):
		self.Tk.destroy()

	def Update(self):
		key = [0,0,0, 0,0,0, 0,0,0, 0]
		while self.Play:
			
			for event in pygame.event.get():
				if event.type == pygame.QUIT:
						self.Play = False	
				if event.type == pygame.KEYDOWN or event.type == pygame.KEYUP:
					if event.key == pygame.K_w:
						key[0]=event.type == pygame.KEYDOWN
					elif event.key == pygame.K_s:
						key[1]=event.type == pygame.KEYDOWN
					elif event.key == pygame.K_a:
						key[2]=event.type == pygame.KEYDOWN
					elif event.key == pygame.K_d:
						key[3]=event.type == pygame.KEYDOWN
					elif event.key == pygame.K_LEFT:
						key[4]=event.type == pygame.KEYDOWN
					elif event.key == pygame.K_RIGHT:
						key[5]=event.type == pygame.KEYDOWN
					elif event.key == pygame.K_UP:
						key[6]=event.type == pygame.KEYDOWN
					elif event.key == pygame.K_DOWN:
						key[7]=event.type == pygame.KEYDOWN
					elif event.key == pygame.K_SPACE:
						key[8]=event.type == pygame.KEYDOWN
					elif event.key == pygame.K_LSHIFT:
						key[9]=event.type == pygame.KEYDOWN
			#self.Canvas.delete("all")
			if key[0]:
				self.Camera.Z += cos(self.Rot)
				self.Camera.X += -sin(self.Rot)
			if key[1]:
				self.Camera.Z += -cos(self.Rot)
				self.Camera.X += sin(self.Rot)
			if key[2]:
				self.Camera.Z += sin(self.Rot)
				self.Camera.X += cos(self.Rot)
			if key[3]:
				self.Camera.Z += -sin(self.Rot)
				self.Camera.X += -cos(self.Rot)
			if key[4]:
				self.Rot-=0.1
			if key[5]:
				self.Rot+=0.1
			if key[6]:
				self.Rot2-=0.1
			if key[7]:
				self.Rot2+=0.1
			if key[8]:
				self.Camera.Y -= 1
			if key[9]:
				self.Camera.Y += 1
			
			Renderer.Clear()
			
			proj = Renderer.GetProjMat()
			
			#proj = proj * Renderer.GetRotMatX(0.3) * Renderer.GetRotMatY(self.Rot) * Renderer.GetTransMat(self.Camera.X, -9, self.Camera.Z)
			#proj = proj * Renderer.GetRotMatX(0.3) * Renderer.GetRotMatY(-pi/8) * Renderer.GetTransMat(-5, -5, -10)
			#proj = proj * Renderer.GetRotMatY(-0.3) * Renderer.GetTransMat(-5, -5, 0) * Renderer.GetTransMat(0,0,-10*cos(self.Rot))
			#self.Camera.X = 12*sin(self.Rot)
			#self.Camera.Z = -12*cos(self.Rot)
			#self.Rot+=0.05
			proj = proj * Renderer.GetRotMatX(self.Rot2) * Renderer.GetRotMatY(self.Rot) * Renderer.GetTransMat(self.Camera.X, self.Camera.Y, self.Camera.Z)
			
			Renderer.Render(Triangle(Vec3(-10,0,0), Vec3(10,0,0), Vec3(0,10,0)), proj,
							Triangle(Vec3(255,0,0), Vec3(0,255,0), Vec3(0,0,255)))
			Renderer.Render(Triangle(Vec3(0,0,0), Vec3(0,0,-10), Vec3(0,10,-10)), proj,
							Triangle(Vec3(255,0,0), Vec3(0,255,0), Vec3(0,0,255)))
			Renderer.Render(Triangle(Vec3(1,0,-1), Vec3(1,0,-10), Vec3(10,0,-1)), proj,
							Triangle(Vec3(255,0,0), Vec3(0,255,0), Vec3(0,0,255)))
			Renderer.Render(Triangle(Vec3(-1,10,-1), Vec3(-1,10,-10), Vec3(-10,10,-1)), proj,
							Triangle(Vec3(255,0,0), Vec3(0,255,0), Vec3(0,0,255)))
			Renderer.Render(Triangle(Vec3(0,10,-10), Vec3(0,10,10), Vec3(10,10,0)), proj,
							Triangle(Vec3(255,0,0), Vec3(0,255,0), Vec3(0,0,255)))
			
			Renderer.Draw(self.Surface, self.Width, self.Height)
			
			
			self.Screen.blit(pygame.transform.scale(self.Surface, (self.Width, self.Height)), (0,0))
			
			pygame.display.flip()
		
		#self.Canvas.update()
		#self.Tk.after(10, self.Update)

for z in (-15,-10,-5,-2,0,2,5,10,15):
	orig = Vec3(5,5,z)
	homog = Renderer.GetProjMat() * Vec3(5,5,z).ToMat()
	print(str(orig) + "-->" + str(Vec3.FromMat(homog)) + "  {" + str(Vec4.FromMat(homog)))

screen = Screen()

