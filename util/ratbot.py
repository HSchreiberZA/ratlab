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

# math
import math
import random as rnd
import cv2

import numpy  as np

# utilities / own
import util.freezeable as freezeable

Freezeable = freezeable.Freezeable

# ------------------------------------------------------------------[ Constants ]

def_RAD2DEG = 180.0 / math.pi
def_DEG2RAD = math.pi / 180.0

# ------------------------------------------------------------------[ Numpy Mod ]

np.seterr(divide='ignore')  # ignore 'division by zero' errors (occur on path reset)


# ==============================================================[ Step Class ]
class Step:
    def __init__(self, is_valid, position, view):
        self.__is_valid__ = is_valid
        self.__position__ = position
        self.__view__ = view


# ==============================================================[ RatBot Class ]

class RatBot(Freezeable):
    """
    Class to set up and control a virtual rodent.
    """

    # ----------------------------------------------------------[ Construction ]

    def __init__(self, control, controller):
        """
        Constructor. Initializes the rat bot.
        pos    : Valid 2D position within the simulation world.
        control: Ai2Thor Controller
        """
        rnd.seed()
        # simulation control panel
        self.__ctrl__ = control
        self.__controller__ = controller
        # path
        event = controller.step(dict(action='Crouch'))
        self.__path__ = []
        self.__path__.append(self.unpack(event.metadata['agent']['position']))
        # follow path if specified via file
        self.freeze()

    @staticmethod
    def unpack(position):
        return np.array([position['x'], position['z']])

    # -----------------------------------------------------------[ Path Control ]

    def get_path(self):
        """
        Retrieve the rat's path data so far. The function returns an array of 2D
        positions.
        """
        return self.__path__

    def __gaussianWhiteNoise2D__(self, dir=None):
        # random unrestricted direction
        if dir is None or self.__ctrl__.setup.rat.arc == 360.0:
            angle = (rnd.random() * 360.0) * def_DEG2RAD
            return np.array([math.cos(angle), math.sin(angle)])
        # random direction focused around given velocity vector
        else:
            try:
                dir_n = dir / math.sqrt(dir[0] ** 2 + dir[1] ** 2)
                dir_a = math.asin(abs(dir_n[1])) * def_RAD2DEG
                if dir_n[0] <= 0 <= dir_n[1]:
                    dir_a = 180.0 - dir_a
                elif dir_n[0] <= 0 and dir_n[1] <= 0:
                    dir_a = 180.0 + dir_a
                elif dir_n[0] >= 0 >= dir_n[1]:
                    dir_a = 360.0 - dir_a
                rat_fov = self.__ctrl__.setup.rat.arc
                angle = (dir_a - rat_fov / 2.0 + rnd.random() * rat_fov) * def_DEG2RAD
                return np.array([math.cos(angle), math.sin(angle)])
            except ValueError:
                # random rebound in case the path gets stuck in a corner
                return self.__gaussianWhiteNoise2D__()

    def followPathNodes(self):
        # switch to next nav point when necessary
        path = self.__ctrl__.setup.rat.path
        pos = self.__path__[len(self.__path__) - 1]
        dist = np.sqrt(np.vdot(pos - path[self.__path_index__], pos - path[self.__path_index__]))
        if dist < self.__ctrl__.setup.rat.speed:
            self.__path_index__ += 1
            self.__path_index__ %= len(path)
            # end of non-loop path: teleport back to starting position
            if self.__path_index__ == 0 and self.__ctrl__.setup.rat.path_loop == False:
                pos_next = path[0]
                trajectory = np.array(path[1] - path[0], dtype=np.float32)
                trajectory /= np.sqrt(np.vdot(trajectory, trajectory))
                self.__path__.append(pos_next)
                return pos_next, trajectory
        # new step
        step = np.array(path[self.__path_index__] - pos, dtype=np.float32)
        step /= np.sqrt(np.vdot(step, step))
        noise = self.__ctrl__.setup.rat.path_dev
        while True:
            if np.random.random() > 0.5:
                step += np.array([-step[1], step[0]]) * noise
            else:
                step += np.array([step[1], -step[0]]) * noise
            step *= self.__ctrl__.setup.rat.speed
            # check for valid step
            pos_next = pos + step
            # if self.__ctrl__.modules.world.validStep( pos, pos_next ) == True:
            self.__path__.append(pos_next)
            return pos_next, step

    def next_path_step(self):
        """
        Generate the next step of the rat's movement.
        """
        # current position & velocity/direction
        pos = self.__path__[len(self.__path__) - 1]
        pos_next = np.array([np.nan, np.nan])
        view = []
        if len(self.__path__) > 1:
            vel = pos - self.__path__[len(self.__path__) - 2]
        else:
            vel = self.__gaussianWhiteNoise2D__()
        # generate next step
        while True:
            noise = self.__gaussianWhiteNoise2D__(vel)
            mom = self.__ctrl__.setup.rat.path_mom
            step = vel * mom + noise * (1.0 - mom)
            step /= np.sqrt(np.vdot(step, step))
            step *= self.__ctrl__.setup.rat.speed
            # optional movement bias
            bias = self.__ctrl__.setup.rat.bias
            step += bias * (np.dot(bias, step) ** 2) * np.sign(np.dot(bias, step)) * self.__ctrl__.setup.rat.bias_s
            # check for valid step
            step = np.array([step[0], step[1]])
            pos_next = np.round(pos + step, 1)
            valid_step = self.__ctrl__.modules.world.valid_step(pos, pos_next)
            if not valid_step.__is_valid__:
                vel *= 0.5
            else:
                pos_next = valid_step.__position__
                resized = cv2.resize(valid_step.__view__, (55, 35), interpolation=cv2.INTER_AREA)
                view = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
                break
        # set and confirm
        self.__path__.append(pos_next)
        return pos_next, pos_next - pos, view
