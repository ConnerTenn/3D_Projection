
from tkinter import *
import time
import random
from MatrixCode import *
from math import *

run = True

class Point:
	def __init__(self, x=0, y=0, z=0, r=0, g=0, b=0):
		self.X = x
		self.Y = y
		self.Z = z

		self.R = r
		self.G = g
		self.B = b
	
	def __add__(self, other):
		return Point(self.X + other.X, self.Y + other.Y, self.Z + other.Y)
	
	def __sub__(self, other):
		return Point(self.X - other.X, self.Y - other.Y, self.Z - other.Y)
	
	def __mul__(self, other):
		return Point(self.X * other.X, self.Y * other.Y, self.Z * other.Y)

	def Mat(self):
		return Matrix.fromList([[self.X],[self.Y],[self.Z],[1]])

	def FromMat(self, mat):
		self.X = mat[0][0]
		self.Y = mat[1][0]
		self.Z = mat[2][0]

	def FromHomog(self, mat):
		if mat[3][0] == 0:
			mat[3][0] = 0.001
		self.X = mat[0][0]/mat[3][0]
		self.Y = mat[1][0]/mat[3][0]
		self.Z = mat[2][0]/mat[3][0]
		
	def Dot2(self, other):
		return self.X * other.X + self.Y * other.Y

	def __str__(self):
		return "["+str(round(self.X,3))+", "+str(round(self.Y,3))+", "+str(round(self.Z,3))+"]"


def PointFromMat(mat):
	point = Point(0,0,0)
	point.X = mat[0][0]
	point.Y = mat[1][0]
	point.Z = mat[2][0]
	return point

def PointFromHomog(mat):
	point = Point(0,0,0)
	if mat[3][0] == 0:
		mat[3][0] = 0.001
	point.X = mat[0][0]/mat[3][0]
	point.Y = mat[1][0]/mat[3][0]
	point.Z = mat[2][0]/mat[3][0]
	return point

def Near(val, rnd):
	return round(val/rnd)*rnd
	
class Triangle:
	def __init__(self, p1=Point(), p2=Point(), p3=Point()):
		self.Points = [p1, p2, p3]
		
		self.v0 = self.Points[1] - self.Points[0]
		self.v1 = self.Points[2] - self.Points[0]
		self.d00 = self.v0.Dot2(self.v0)
		self.d01 = self.v0.Dot2(self.v1)
		self.d11 = self.v1.Dot2(self.v1)
		denom = (self.d00 * self.d11 - self.d01 * self.d01)
		self.invdenom = 1 / denom if not denom == 0 else 1000000
	
	def Clockwise(self):
		None
		
	def __str__(self):
		return "{ " + str(self.Points[0]) + " " + str(self.Points[1]) + " " + str(self.Points[2]) + " }"
		
	def GenerateLines(self):
		self.a12 = -(self.Points[1].Y-self.Points[0].Y)
		self.b12 = (self.Points[1].X-self.Points[0].X)
		self.c12 = self.a12 * self.Points[0].X + self.b12 * self.Points[0].Y
		
		self.a13 = -(self.Points[2].Y-self.Points[0].Y)
		self.b13 = (self.Points[2].X-self.Points[0].X)
		self.c13 = self.a13 * self.Points[0].X + self.b13 * self.Points[0].Y
		
		self.a23 = -(self.Points[2].Y-self.Points[1].Y)
		self.b23 = (self.Points[2].X-self.Points[1].X)
		self.c23 = self.a23 * self.Points[1].X + self.b23 * self.Points[1].Y
		
	def ContainsPoint(self, point, near):
		if self.a12 * Near(point.X, near) + self.b12 * Near(point.Y, near) < self.c12 and \
			self.a13 * Near(point.X, near) + self.b13 * Near(point.Y, near) > self.c13 and \
			self.a23 * Near(point.X, near) + self.b23 * Near(point.Y, near) < self.c23:
			return True
		return False

class Renderer:
	def __init__(self, canvas, width=500, height=300, fov=pi/2, cam=Point()):
		self.Canvas = canvas
		self.Width = width
		self.Height = height
		self.FOV = fov
		self.rot = 0
		self.Q=0
		self.Camera = cam
		
		self.Resize()

	def Resize(self):
		self.PixelBuff = [[[0 for i in range(3)] for y in range(self.Height)] for x in range(self.Width)]
		self.DepthBuf = [[0 for y in range(self.Height)] for x in range(self.Width)]

	@staticmethod
	def GetProjectionMatrix():
		n=1; f=100; r=3; t=3
		return Matrix.fromList(                                 \
			[                                                   \
				[ n/r,   0,     0,             0             ], \
				[ 0,     n/t,   0,             0             ], \
				[ 0,     0,    -(f+n)/(f-n),  -(2*f*n)/(f-n) ], \
				[ 0,     0,    -1,             0             ]  \
			])
	
	@staticmethod
	def GetTranslationMatrix(x, y, z):
		return Matrix.fromList(
			[                          \
				[  1,   0,   0,   x ], \
				[  0,   1,   0,   y ], \
				[  0,   0,   1,   z ], \
				[  0,   0,   0,   1 ]  \
			])

	@staticmethod
	def GetRotationMatrixX(ang):
		return Matrix.fromList(                      \
			[                                        \
				[  1,   0,          0,          0 ], \
				[  0,   cos(ang),  -sin(ang),   0 ], \
				[  0,   sin(ang),   cos(ang),   0 ], \
				[  0,   0,           0      ,   1 ]  \
			])

	@staticmethod
	def GetRotationMatrixY(ang):
		return Matrix.fromList(                      \
			[                                        \
				[  cos(ang),   0,   sin(ang),   0 ], \
				[  0,          1,   0,          0 ], \
				[ -sin(ang),   0,   cos(ang),   0 ], \
				[  0,          0,   0,          1 ]  \
			])

	@staticmethod
	def GetRotationMatrixZ(ang):
		return Matrix.fromList(                      \
			[                                        \
				[  cos(ang),  -sin(ang),   0,   0 ], \
				[  sin(ang),   cos(ang),   0,   0 ], \
				[  1,          0,          1,   0 ], \
				[  0,          0,          0,   1 ]  \
			])

	def Shader(self):
		None
		
	def ToScreenCoords(self, point):
		return (point*Point(1,-1,1)+Point(0.5,0.5,0))*Point(1000,600,1)

	def Render(self, tri):
		
		self.Camera.X = 12*sin(self.rot)
		self.Camera.Z = -12*cos(self.rot)
		self.rot+=0.003
		
		proj = self.GetProjectionMatrix()
		triProj = Triangle()
		#print("TRI:\n")
		for i in range(3):
			#print(tri.Points[i].Mat())
			#triProj.Points[i].FromHomog( proj * self.GetTranslationMatrix(self.Camera.X, 0, self.Camera.Z)  * tri.Points[i].Mat())
			triProj.Points[i].FromHomog( proj * Renderer.GetRotationMatrixX(0.3) * Renderer.GetRotationMatrixY(self.rot) * Renderer.GetTranslationMatrix(self.Camera.X, -9, self.Camera.Z) * tri.Points[i].Mat())
		
		p1 = self.ToScreenCoords(triProj.Points[0])
		p2 = self.ToScreenCoords(triProj.Points[1])
		p3 = self.ToScreenCoords(triProj.Points[2])
		
		
		#print(str(self.Camera) + "    " + str(round(self.rot/(2*pi),3)))
		q=['black', 'red', 'green','blue']
		self.Q=(self.Q+1)%4
		print(str(triProj.Points[0].Z))
		self.Canvas.create_polygon(p1.X, p1.Y, p2.X, p2.Y, p3.X, p3.Y, fill=q[self.Q])
		
		d=max(min(floor(255*(1-triProj.Points[0].Z)/2), 255), 0)
		colorval = "#%02x%02x%02x" % (d, d, d)
		self.Canvas.create_oval(p1.X-10, p1.Y-10, p1.X+10, p1.Y+10, fill=colorval, outline="")
		
		
		d=max(min(floor(255*(1-triProj.Points[1].Z)/2), 255), 0)
		colorval = "#%02x%02x%02x" % (d, d, d)
		self.Canvas.create_oval(p2.X-10, p2.Y-10, p2.X+10, p2.Y+10, fill=colorval, outline="")
		
		
		d=max(min(floor(255*(1-triProj.Points[2].Z)/2), 255), 0)
		colorval = "#%02x%02x%02x" % (d, d, d)
		self.Canvas.create_oval(p3.X-10, p3.Y-10, p3.X+10, p3.Y+10, fill=colorval, outline="")

def Dist(x1,y1,x2,y2):
	return sqrt(pow(x2-x1,2) + pow(y2-y1,2))

'''void Barycentric(Point p, Point a, Point b, Point c, float &u, float &v, float &w)
{
    Vector v0 = b - a, v1 = c - a, v2 = p - a;
    float d00 = Dot(v0, v0);
    float d01 = Dot(v0, v1);
    float d11 = Dot(v1, v1);
    float d20 = Dot(v2, v0);
    float d21 = Dot(v2, v1);
    float denom = d00 * d11 - d01 * d01;
    v = (d11 * d20 - d01 * d21) / denom;
    w = (d00 * d21 - d01 * d20) / denom;
    u = 1.0f - v - w;
}'''

def Interp(p, tri, val):
	#v0 = tri.Points[1] - tri.Points[0]
	#v1 = tri.Points[2] - tri.Points[0]
	v2 = p - tri.Points[0]
	#d00 = v0.Dot2(v0)
	#d01 = v0.Dot2(v1)
	#d11 = v1.Dot2(v1)
	d20 = v2.Dot2(tri.v0)
	d21 = v2.Dot2(tri.v1)
	#denom = d00 * d11 - d01 * d01
	#v = (d11 * d20 - d01 * d21) / denom
	#w = (d00 * d21 - d01 * d20) / denom
	v = (tri.d11 * d20 - tri.d01 * d21) * tri.invdenom
	w = (tri.d00 * d21 - tri.d01 * d20) * tri.invdenom
	u = 1 - (v + w)

	'''v = 0 if v < 0 else v
	w = 0 if w < 0 else w
	u = 0 if u < 0 else u

	v = 1 if v > 1 else v
	w = 1 if w > 1 else w
	u = 1 if u > 1 else u'''
	
	ret = val[0] * u + val[1] * v + val[2] * w
	mx = val[0] + val[1] + val[2]
	ret = mx if ret > mx else ret
	ret = 0 if ret < 0 else ret
	return ret

class Screen:
	def __init__(self, Width=1000, Height=600, Title="Projection"):
		self.Width=Width
		self.Height=Height
		self.Title=Title

		self.Play=True

		self.tk=Tk()
		self.tk.title(self.Title)
		self.tk.resizable(0,0)
		self.canvas=Canvas(self.tk, width=self.Width, height=self.Height)
		self.canvas.pack()
		
		self.tk.protocol("WM_DELETE_WINDOW", self.Close)
		
		self.Renderer = Renderer(self.canvas)

		self.Update()
		self.tk.mainloop()
	
	def Close(self):
		self.tk.destroy()

	def Update(self):
		self.canvas.delete("all")
		
		self.Renderer.Render(Triangle(Point(-10,0,0), Point(10,0,0), Point(0,10,0)))
		self.Renderer.Render(Triangle(Point(0,0,0), Point(0,0,-10), Point(0,10,-10)))
		self.Renderer.Render(Triangle(Point(1,0,-1), Point(1,0,-10), Point(10,0,-1)))
		self.Renderer.Render(Triangle(Point(-1,10,-1), Point(-1,10,-10), Point(-10,10,-1)))
		'''
		tri = Triangle(Point(100,100,0), Point(300,500,0), Point(500,100,0))
		tri.GenerateLines()
		colours = Triangle(Point(255,0,0), Point(0,255,0), Point(0,0,255))

		for y in range(0, floor(self.Height/10)):
			for x in range(0, floor(self.Width/10)):
				if tri.ContainsPoint(Point(x*10,y*10), 10):
					R = Interp(Point(x*10,y*10), tri, [colours.Points[0].X, colours.Points[1].X, colours.Points[2].X])
					G = Interp(Point(x*10,y*10), tri, [colours.Points[0].Y, colours.Points[1].Y, colours.Points[2].Y])
					B = Interp(Point(x*10,y*10), tri, [colours.Points[0].Z, colours.Points[1].Z, colours.Points[2].Z])
					
					colorval = "#%02x%02x%02x" % (floor(R), floor(G), floor(B))
					self.canvas.create_rectangle(x*10, y*10, x*10+10, y*10+10, fill=colorval, outline="")
		'''
		self.canvas.update()
		self.tk.after(10, self.Update)

#a = Matrix.fromList([[1],[2]])
#b = Matrix.fromList([[3, 4]])
#print(str(a) + "\nTimes\n" + str(b) + "\nEquals\n" + str(a*b))

screen = Screen()
'''
Renderer = Renderer(0)
Proj = Renderer.GetRotationMatrixY(pi*0.0) * Renderer.GetProjectionMatrix()
Point1 = Point(20,20,-20)

#worldMat = ScaleMat * rotationMat * translationMat
Point1Homog = Proj * Point1.Mat()
Point1Proj = PointFromHomog(Point1Homog)
print(str( Point1Homog ))
print(str( Point1Proj ))
'''

