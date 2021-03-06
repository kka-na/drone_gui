"""
pip install PyOpenGL PyOpenGL_accelerate
https://github.com/pyside/Examples/blob/master/examples/opengl/samplebuffers.py
"""
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import sys, math

import myobjloader as myloader

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QOpenGLWidget
import controlros

class ImuGL(QOpenGLWidget) :
	def  __init__(self,  objname, parent = None,):
		super(ImuGL, self).__init__(parent)
		self.ctrlros = controlros.ControlRos()
		self.ctrlros.send_imu.connect(self.update_imu)
		self.obj = objname
		self.roll = 0.0
		self.pitch = 0.0
		self.yaw = 0.0

	def initializeGL(self):
		glutInit()
		glLightfv(GL_LIGHT0, GL_POSITION, (-500, 600, 500, 1.0))
		glLightfv(GL_LIGHT0, GL_AMBIENT, (0.7, 0.7, 0.7, 1.0))
		glLightfv(GL_LIGHT0, GL_DIFFUSE, (0.5, 0.5, 0.5, 1.0))
		glEnable(GL_LIGHT0)
		glEnable(GL_LIGHTING)
		glEnable(GL_DEPTH_TEST)
		glEnable(GL_COLOR_MATERIAL)
		glShadeModel(GL_SMOOTH)

		glClearColor(0.15,0.16,0.21,1.0)
		glMatrixMode(GL_PROJECTION)
		glLoadIdentity()
		gluPerspective(-100, 300, 30, 100.0)
		glEnable(GL_DEPTH_TEST)
		glMatrixMode(GL_MODELVIEW)
		self.car = myloader.ObjLoader(self.obj)
		

	def paintGL(self):
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
		glLoadIdentity()
		glPushMatrix()
		glScalef(1.0,1.0,1.0)
		glTranslatef(0.0,0.0,-5.5)
		glRotatef(30,1,0,0)
		glRotatef(-45,0,1,0)

		self.drawLine()

		glRotatef(self.roll, 1, 0, 0)
		glRotatef(self.pitch, 0, 1, 0)
		glCallList(self.car.gl_list)
		glPopMatrix()
	
	@pyqtSlot(float, float, float)
	def update_imu(self, roll_change, pitch_change, yaw_change):
		self.roll = roll_change
		self.pitch = pitch_change
		self.yaw = yaw_change
		self.update()

	def drawLine(self) :
		xroll = "R_{}".format(round(self.roll,3))
		ypitch = "P_{}".format(round(self.pitch,3))
		zyaw = "Y_{}".format(round(self.yaw,3))

		glPushMatrix()

		glDisable(GL_LIGHTING)
		glPushMatrix();
		glColor4f(0.313, 0.737, 0.929,1.0)
		glBegin(GL_LINES)
		glVertex3f(5.0, 0.0, 0.0)
		glVertex3f(-5.0, 0.0, 0.0)
		glEnd()
		self.drawBitmapText(xroll, -5.0, 0.0, 0.0)
		glPopMatrix()

		glPushMatrix()
		glColor4f(0.313, 0.929, 0.615,1.0)
		glBegin(GL_LINES)
		glVertex3f(0.0, 5.0, 0.0)
		glVertex3f(0.0, -5.0, 0.0)
		glEnd()
		self.drawBitmapText(ypitch, 0.0, 1.9,-0.2)
		glPopMatrix()
		

		glPushMatrix()
		glColor4f(0.823, 0.929, 0.313, 1.0)
		glBegin(GL_LINES)
		glVertex3f(0.0, 0.0, 5.0)
		glVertex3f(0.0, 0.0, -5.0)
		glEnd()
		self.drawBitmapText(zyaw, 0.0, 0.3, 2.3)
		glPopMatrix()

		glEnable(GL_LIGHTING)
		glPopMatrix()
		glFlush()

	def drawBitmapText(self, str, x, y, z) :
		glRasterPos3f(x, y, z)
		for ch in str :
			glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, ctypes.c_int(ord(ch)))

	def resizeGL(self, width, height):
		glGetError()
		aspect = width if (height == 0) else width / height
		glViewport(0, 0, width,height)
		glMatrixMode(GL_PROJECTION)
		glLoadIdentity()
		gluPerspective(45, aspect, 1, 100.0)
		glMatrixMode(GL_MODELVIEW)
		glLoadIdentity()
