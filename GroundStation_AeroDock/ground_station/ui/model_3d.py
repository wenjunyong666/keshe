# -*- coding: utf-8 -*-
"""
3D模型显示组件
"""

from PyQt5.QtWidgets import QWidget, QVBoxLayout
from PyQt5.QtCore import QTimer
from OpenGL.GL import *
from OpenGL.GLU import *
from PyQt5.QtOpenGL import QGLWidget
import math
import numpy as np


class Model3DWidget(QGLWidget):
    """3D模型显示组件"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.zoom = -10.0

    def initializeGL(self):
        """初始化OpenGL"""
        glClearColor(0.07, 0.13, 0.11, 1.0)  # AeroDock 深绿仪表舱背景
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)

        # 设置光源
        glLightfv(GL_LIGHT0, GL_POSITION, [1.0, 1.0, 1.0, 0.0])
        glLightfv(GL_LIGHT0, GL_AMBIENT, [0.35, 0.45, 0.42, 1.0])
        glLightfv(GL_LIGHT0, GL_DIFFUSE, [1.0, 0.82, 0.48, 1.0])

    def resizeGL(self, w, h):
        """调整视口"""
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45.0, w / h if h != 0 else 1, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)

    def paintGL(self):
        """绘制场景"""
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()

        # 设置相机位置
        glTranslatef(0.0, 0.0, self.zoom)

        # 应用姿态旋转（注意顺序：Yaw -> Pitch -> Roll）
        # Convert FC aircraft-axis attitude to this OpenGL model frame.
        # Numeric cards still show the raw protocol values; only the 3D view is adapted.
        glRotatef(-self.yaw, 0.0, 1.0, 0.0)
        glRotatef(-self.pitch, 1.0, 0.0, 0.0)
        glRotatef(-self.roll, 0.0, 0.0, 1.0)

        # 绘制飞机模型
        self.draw_aircraft()

        # 绘制坐标轴
        self.draw_axes()

    def draw_aircraft(self):
        """绘制精细的四轴无人机模型"""
        # 中心机身 - 圆形主体
        glPushMatrix()
        glColor3f(0.05, 0.20, 0.17)
        self.draw_cylinder(0.4, 0.25, 32)
        glPopMatrix()

        # 机身顶盖
        glPushMatrix()
        glTranslatef(0.0, 0.15, 0.0)
        glColor3f(0.10, 0.35, 0.30)
        self.draw_sphere(0.35, 24, 24)
        glPopMatrix()

        # 机身底部电池仓
        glPushMatrix()
        glTranslatef(0.0, -0.2, 0.0)
        glColor3f(0.95, 0.60, 0.22)
        glScalef(0.5, 0.15, 0.3)
        self.draw_cube()
        glPopMatrix()

        # 机臂位置（X型布局，45度角）
        arm_angles = [45, 135, 225, 315]

        # 电机颜色（前红后白，左右对称）
        motor_colors = [
            (0.94, 0.43, 0.32),  # 右前
            (0.94, 0.43, 0.32),  # 左前
            (0.36, 0.72, 0.66),  # 左后
            (0.36, 0.72, 0.66)   # 右后
        ]

        # LED灯颜色
        led_colors = [
            (1.0, 0.76, 0.32),   # 右前
            (1.0, 0.76, 0.32),   # 左前
            (0.40, 0.95, 0.86),  # 左后
            (0.40, 0.95, 0.86)   # 右后
        ]

        for i, angle in enumerate(arm_angles):
            glPushMatrix()
            glRotatef(angle, 0.0, 1.0, 0.0)

            # 机臂 - 圆柱形
            glPushMatrix()
            glTranslatef(0.8, 0.0, 0.0)
            glRotatef(90, 0.0, 0.0, 1.0)
            glColor3f(0.18, 0.42, 0.36)
            self.draw_cylinder(0.06, 1.0, 16)
            glPopMatrix()

            # 机臂末端加强
            glPushMatrix()
            glTranslatef(1.3, 0.0, 0.0)
            glColor3f(0.3, 0.3, 0.3)
            self.draw_sphere(0.1, 16, 16)
            glPopMatrix()

            # 电机底座
            glPushMatrix()
            glTranslatef(1.3, -0.1, 0.0)
            glColor3f(0.2, 0.2, 0.2)
            self.draw_cylinder(0.12, 0.08, 16)
            glPopMatrix()

            # 电机主体
            glPushMatrix()
            glTranslatef(1.3, -0.18, 0.0)
            glColor3f(*motor_colors[i])
            self.draw_cylinder(0.15, 0.25, 24)
            glPopMatrix()

            # 电机轴
            glPushMatrix()
            glTranslatef(1.3, -0.05, 0.0)
            glColor3f(0.5, 0.5, 0.5)
            self.draw_cylinder(0.02, 0.15, 8)
            glPopMatrix()

            # 螺旋桨（双叶）
            glPushMatrix()
            glTranslatef(1.3, 0.05, 0.0)
            glRotatef(90, 1.0, 0.0, 0.0)
            glColor3f(0.1, 0.1, 0.1)
            self.draw_propeller_detailed()
            glPopMatrix()

            # LED指示灯
            glPushMatrix()
            glTranslatef(1.3, 0.0, 0.0)
            glColor3f(*led_colors[i])
            self.draw_sphere(0.04, 12, 12)
            glPopMatrix()

            glPopMatrix()

        # 顶部GPS/指示灯
        glPushMatrix()
        glTranslatef(0.0, 0.35, 0.0)
        glColor3f(0.0, 0.8, 1.0)
        self.draw_cylinder(0.08, 0.05, 16)
        glPopMatrix()

        # 底部摄像头云台
        glPushMatrix()
        glTranslatef(0.0, -0.4, 0.15)
        glColor3f(0.1, 0.1, 0.1)
        self.draw_sphere(0.12, 16, 16)
        glPopMatrix()

        # 摄像头镜头
        glPushMatrix()
        glTranslatef(0.0, -0.4, 0.25)
        glColor3f(0.05, 0.05, 0.05)
        self.draw_cylinder(0.06, 0.08, 16)
        glPopMatrix()

        # 起落架
        for angle in [30, 150, 210, 330]:
            glPushMatrix()
            glRotatef(angle, 0.0, 1.0, 0.0)
            glTranslatef(0.3, -0.35, 0.0)
            glColor3f(0.3, 0.3, 0.3)
            self.draw_cylinder(0.02, 0.3, 8)
            glPopMatrix()

    def draw_cube(self):
        """绘制立方体"""
        glBegin(GL_QUADS)

        # 前面
        glNormal3f(0.0, 0.0, 1.0)
        glVertex3f(-0.5, -0.5, 0.5)
        glVertex3f(0.5, -0.5, 0.5)
        glVertex3f(0.5, 0.5, 0.5)
        glVertex3f(-0.5, 0.5, 0.5)

        # 后面
        glNormal3f(0.0, 0.0, -1.0)
        glVertex3f(-0.5, -0.5, -0.5)
        glVertex3f(-0.5, 0.5, -0.5)
        glVertex3f(0.5, 0.5, -0.5)
        glVertex3f(0.5, -0.5, -0.5)

        # 顶面
        glNormal3f(0.0, 1.0, 0.0)
        glVertex3f(-0.5, 0.5, -0.5)
        glVertex3f(-0.5, 0.5, 0.5)
        glVertex3f(0.5, 0.5, 0.5)
        glVertex3f(0.5, 0.5, -0.5)

        # 底面
        glNormal3f(0.0, -1.0, 0.0)
        glVertex3f(-0.5, -0.5, -0.5)
        glVertex3f(0.5, -0.5, -0.5)
        glVertex3f(0.5, -0.5, 0.5)
        glVertex3f(-0.5, -0.5, 0.5)

        # 右面
        glNormal3f(1.0, 0.0, 0.0)
        glVertex3f(0.5, -0.5, -0.5)
        glVertex3f(0.5, 0.5, -0.5)
        glVertex3f(0.5, 0.5, 0.5)
        glVertex3f(0.5, -0.5, 0.5)

        # 左面
        glNormal3f(-1.0, 0.0, 0.0)
        glVertex3f(-0.5, -0.5, -0.5)
        glVertex3f(-0.5, -0.5, 0.5)
        glVertex3f(-0.5, 0.5, 0.5)
        glVertex3f(-0.5, 0.5, -0.5)

        glEnd()

    def draw_axes(self):
        """绘制坐标轴"""
        glDisable(GL_LIGHTING)
        glLineWidth(2.0)
        glBegin(GL_LINES)

        # X轴 - 红色
        glColor3f(1.0, 0.0, 0.0)
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(2.0, 0.0, 0.0)

        # Y轴 - 绿色
        glColor3f(0.0, 1.0, 0.0)
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(0.0, 2.0, 0.0)

        # Z轴 - 蓝色
        glColor3f(0.0, 0.0, 1.0)
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(0.0, 0.0, 2.0)

        glEnd()
        glEnable(GL_LIGHTING)

    def draw_cylinder(self, radius: float, height: float, slices: int = 16):
        """绘制圆柱体"""
        glBegin(GL_QUAD_STRIP)
        for i in range(slices + 1):
            angle = 2.0 * math.pi * i / slices
            x = radius * math.cos(angle)
            z = radius * math.sin(angle)
            glNormal3f(x, 0.0, z)
            glVertex3f(x, -height/2, z)
            glVertex3f(x, height/2, z)
        glEnd()

        # 顶盖
        glBegin(GL_TRIANGLE_FAN)
        glNormal3f(0.0, 1.0, 0.0)
        glVertex3f(0.0, height/2, 0.0)
        for i in range(slices + 1):
            angle = 2.0 * math.pi * i / slices
            x = radius * math.cos(angle)
            z = radius * math.sin(angle)
            glVertex3f(x, height/2, z)
        glEnd()

        # 底盖
        glBegin(GL_TRIANGLE_FAN)
        glNormal3f(0.0, -1.0, 0.0)
        glVertex3f(0.0, -height/2, 0.0)
        for i in range(slices, -1, -1):
            angle = 2.0 * math.pi * i / slices
            x = radius * math.cos(angle)
            z = radius * math.sin(angle)
            glVertex3f(x, -height/2, z)
        glEnd()

    def draw_sphere(self, radius: float, slices: int = 16, stacks: int = 16):
        """绘制球体"""
        for i in range(stacks):
            lat0 = math.pi * (-0.5 + float(i) / stacks)
            z0 = radius * math.sin(lat0)
            zr0 = radius * math.cos(lat0)

            lat1 = math.pi * (-0.5 + float(i + 1) / stacks)
            z1 = radius * math.sin(lat1)
            zr1 = radius * math.cos(lat1)

            glBegin(GL_QUAD_STRIP)
            for j in range(slices + 1):
                lng = 2 * math.pi * float(j) / slices
                x = math.cos(lng)
                y = math.sin(lng)

                glNormal3f(x * zr0, y * zr0, z0)
                glVertex3f(x * zr0, y * zr0, z0)
                glNormal3f(x * zr1, y * zr1, z1)
                glVertex3f(x * zr1, y * zr1, z1)
            glEnd()

    def draw_propeller(self):
        """绘制螺旋桨"""
        # 两片桨叶
        for angle in [0, 180]:
            glPushMatrix()
            glRotatef(angle, 0.0, 0.0, 1.0)

            # 桨叶
            glBegin(GL_TRIANGLES)
            glNormal3f(0.0, 0.0, 1.0)
            glVertex3f(0.0, 0.0, 0.0)
            glVertex3f(0.5, 0.1, 0.02)
            glVertex3f(0.5, -0.1, 0.02)
            glEnd()

            glPopMatrix()

    def draw_propeller_detailed(self):
        """绘制精细螺旋桨"""
        # 中心轴
        glColor3f(0.3, 0.3, 0.3)
        self.draw_cylinder(0.03, 0.02, 12)

        # 两片桨叶
        for angle in [0, 180]:
            glPushMatrix()
            glRotatef(angle, 0.0, 0.0, 1.0)

            # 桨叶根部
            glBegin(GL_QUADS)
            glNormal3f(0.0, 0.0, 1.0)
            glVertex3f(0.05, -0.05, 0.01)
            glVertex3f(0.15, -0.08, 0.01)
            glVertex3f(0.15, 0.08, 0.01)
            glVertex3f(0.05, 0.05, 0.01)
            glEnd()

            # 桨叶中部
            glBegin(GL_QUADS)
            glNormal3f(0.0, 0.0, 1.0)
            glVertex3f(0.15, -0.08, 0.015)
            glVertex3f(0.35, -0.1, 0.02)
            glVertex3f(0.35, 0.1, 0.02)
            glVertex3f(0.15, 0.08, 0.015)
            glEnd()

            # 桨叶尖部
            glBegin(GL_TRIANGLES)
            glNormal3f(0.0, 0.0, 1.0)
            glVertex3f(0.35, -0.1, 0.02)
            glVertex3f(0.5, 0.0, 0.025)
            glVertex3f(0.35, 0.1, 0.02)
            glEnd()

            glPopMatrix()

    def update_attitude(self, roll: float, pitch: float, yaw: float):
        """更新姿态角度"""
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.updateGL()
