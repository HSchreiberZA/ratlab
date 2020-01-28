# ==============================================================================
#
#  Copyright (C) 2016 Fabian Schoenfeld
#
#  This file is part of the ratlab software. It is free software; you can
#  redistribute it and/or modify it under the terms of the GNU General Public
#  License as published by the Free Software Foundation; either version 3, or
#  (at your option) any later version.
#
#  This library is distributed in the hope that it will be useful, but WITHOUT
#  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
#  FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
#  more details.
#
#  You should have received a copy of the GNU General Public License along with
#  a special exception for linking and compiling against the pe library, the
#  so-called "runtime exception"; see the file COPYING. If not, see:
#  http://www.gnu.org/licenses/
#
# ==============================================================================


# ======================================================================[ Setup ]
import os
import sys
import time
import types
import string

# python image library
from PIL import Image as img

# math
import math
import numpy
import random as rnd

# OpenGL
from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL import *

# utilities / own
from util.setup import *
from util.opengl_text import *
from util.ratbot import Step

# defines
def_MARKER_HEIGHT = 0.1  # default height of drawn debug markers
def_CUSTOM_HEIGHT = 12.0  # default height for walls in custom mazes ### adapted for epuck scenario atm


# ================================================================[ A Whole New World Class ]

class NewWorld:
    def __init__(self, control, controller, floor_plan, grid_size):
        self.__controller__ = controller
        self.__ctrl__ = control
        controller.start()

        # FloorPlan203 == 208
        # FloorPlan201
        # FloorPlan205
        # FloorPlan209 is cool
        # FloorPlan230

        # FloorPlan207 is like 202
        controller.reset(floor_plan)
        controller.step(dict(action='Initialize', gridSize=grid_size))
        self.__reachable_positions__, self.__limits__ = self.normalize_reachable_positions()

    @staticmethod
    def angle_between(p1, p2):
        angle = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
        angle = angle * (180 / math.pi)
        if angle < 0:
            return 360 - (-angle)
        return angle

    def normalize_reachable_positions(self):
        """
            Round normalized positions so that they match the steps
        """
        reachable_positions = self.__controller__.step(dict(action='GetReachablePositions')).metadata[
            'reachablePositions']
        normalized_positions = []
        limits = numpy.zeros(4)
        for reachable_position in reachable_positions:
            normalized_positions.append({'x': round(reachable_position['x'], 1), 'z': round(reachable_position['z'], 1)})
            if round(reachable_position['x'], 1) < limits[0]:
                limits[0] = round(reachable_position['x'], 1)
            if round(reachable_position['x'], 1) > limits[2]:
                limits[2] = round(reachable_position['x'], 1)
            if round(reachable_position['z'], 1) < limits[1]:
                limits[1] = round(reachable_position['z'], 1)
            if round(reachable_position['z'], 1) > limits[3]:
                limits[3] = round(reachable_position['z'], 1)
        return normalized_positions, numpy.array(limits * 10).astype(int)

    def valid_step(self, pos, pos_new):
        dist = self.angle_between(pos, pos_new)
        self.__controller__.step(dict(action='Rotate', rotation=dist))
        position_dict = {'x': pos_new[0], 'z': pos_new[1]}

        if position_dict in self.__reachable_positions__:
            return self.teleport(dist, pos_new)
        return Step(False, pos, None)

    def teleport(self, dir, pos_new):
        #self.__controller__.step(dict(action='Rotate', rotation=dir))
        tele = self.__controller__.step(dict(action='TeleportFull', x=pos_new[0], y=0.9026567, z=pos_new[1], rotation=dir, horizon=0.0))
        event = self.__controller__.step(dict(action='Crouch'))
        return Step(tele.metadata['lastActionSuccess'] is True, pos_new, event.cv2img)

    def sketch_path(self, path):
        glColor(self.__ctrl__.color_rat_path)
        glBegin(GL_POINTS)
        for p in path:
            glVertex3f(p[0], p[1], def_MARKER_HEIGHT)
        glEnd()

    @staticmethod
    def sketch_arrow(pos_x, pos_y, dir_x, dir_y, color=None):
        # color
        if color is None:
            glColor(0.6, 0.0, 0.0)
        elif color == 'red':
            glColor(0.8, 0.0, 0.0)
        elif color == 'green':
            glColor(0.0, 0.6, 0.0)
        elif color == 'blue':
            glColor(0.0, 0.0, 0.8)
        elif color == 'grey':
            glColor(0.4, 0.4, 0.4)
        # normalized direction x2
        dir_xn = (dir_x / numpy.sqrt(dir_x ** 2 + dir_y ** 2)) * 2.0
        dir_yn = (dir_y / numpy.sqrt(dir_x ** 2 + dir_y ** 2)) * 2.0
        # draw
        glBegin(GL_LINES)
        glVertex3f(pos_x + dir_xn * 3.0, pos_y + dir_yn * 3.0, def_MARKER_HEIGHT)
        glVertex3f(pos_x + dir_yn, pos_y - dir_xn, def_MARKER_HEIGHT)
        glVertex3f(pos_x + dir_yn, pos_y - dir_xn, def_MARKER_HEIGHT)
        glVertex3f(pos_x - dir_yn, pos_y + dir_xn, def_MARKER_HEIGHT)
        glVertex3f(pos_x - dir_yn, pos_y + dir_xn, def_MARKER_HEIGHT)
        glVertex3f(pos_x + dir_xn * 3.0, pos_y + dir_yn * 3.0, def_MARKER_HEIGHT)
        glVertex3f(pos_x, pos_y, def_MARKER_HEIGHT)
        glVertex3f(pos_x - dir_xn * 2.0, pos_y - dir_yn * 2.0, def_MARKER_HEIGHT)
        glEnd()
