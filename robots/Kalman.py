#from numpy.matlib import matrix
from defines import *
from robot_controller import RobotController
import math

from math import *

import random
class matrix:

    # implements basic operations of a matrix class

    def __init__(self, value):
        self.value = value
        self.dimx = len(value)
        self.dimy = len(value[0])
        if value == [[]]:
            self.dimx = 0

    def zero(self, dimx, dimy):
        # check if valid dimensions
        if dimx < 1 or dimy < 1:
            raise ValueError, "Invalid size of matrix"
        else:
            self.dimx = dimx
            self.dimy = dimy
            self.value = [[0 for row in range(dimy)] for col in range(dimx)]

    def identity(self, dim):
        # check if valid dimension
        if dim < 1:
            raise ValueError, "Invalid size of matrix"
        else:
            self.dimx = dim
            self.dimy = dim
            self.value = [[0 for row in range(dim)] for col in range(dim)]
            for i in range(dim):
                self.value[i][i] = 1

    def show(self):
        for i in range(self.dimx):
            print self.value[i]
        print ' '

    def __add__(self, other):
        # check if correct dimensions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise ValueError, "Matrices must be of equal dimensions to add"
        else:
            # add if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] + other.value[i][j]
            return res

    def __sub__(self, other):
        # check if correct dimensions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise ValueError, "Matrices must be of equal dimensions to subtract"
        else:
            # subtract if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] - other.value[i][j]
            return res

    def __mul__(self, other):
        # check if correct dimensions
        if self.dimy != other.dimx:
            raise ValueError, "Matrices must be m*n and n*p to multiply"
        else:
            # subtract if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, other.dimy)
            for i in range(self.dimx):
                for j in range(other.dimy):
                    for k in range(self.dimy):
                        res.value[i][j] += self.value[i][k] * other.value[k][j]
            return res

    def transpose(self):
        # compute transpose
        res = matrix([[]])
        res.zero(self.dimy, self.dimx)
        for i in range(self.dimx):
            for j in range(self.dimy):
                res.value[j][i] = self.value[i][j]
        return res

    # Thanks to Ernesto P. Adorio for use of Cholesky and CholeskyInverse functions

    def Cholesky(self, ztol=1.0e-5):
        # Computes the upper triangular Cholesky factorization of
        # a positive definite matrix.
        res = matrix([[]])
        res.zero(self.dimx, self.dimx)

        for i in range(self.dimx):
            S = sum([(res.value[k][i])**2 for k in range(i)])
            d = self.value[i][i] - S
            if abs(d) < ztol:
                res.value[i][i] = 0.0
            else:
                if d < 0.0:
                    raise ValueError, "Matrix not positive-definite"
                res.value[i][i] = sqrt(d)
            for j in range(i+1, self.dimx):
                S = sum([res.value[k][i] * res.value[k][j] for k in range(self.dimx)])
                if abs(S) < ztol:
                    S = 0.0
                res.value[i][j] = (self.value[i][j] - S)/res.value[i][i]
        return res

    def CholeskyInverse(self):
        # Computes inverse of matrix given its Cholesky upper Triangular
        # decomposition of matrix.
        res = matrix([[]])
        res.zero(self.dimx, self.dimx)

        # Backward step for inverse.
        for j in reversed(range(self.dimx)):
            tjj = self.value[j][j]
            S = sum([self.value[j][k]*res.value[j][k] for k in range(j+1, self.dimx)])
            res.value[j][j] = 1.0/tjj**2 - S/tjj
            for i in reversed(range(j)):
                res.value[j][i] = res.value[i][j] = -sum([self.value[i][k]*res.value[k][j] for k in range(i+1, self.dimx)])/self.value[i][i]
        return res

    def inverse(self):
        aux = self.Cholesky()
        res = aux.CholeskyInverse()
        return res

    def __repr__(self):
        return repr(self.value)
    def __getitem__(self, item):
        return self.value[item]


class KRC(RobotController):
    STATE_MEASURMENTS = 0;
    STATE_MOTION = 1;
    STATE_DECIDE_NEXT = 2;
    STATE_SCANNING = 3;

    def init(self, starting_position, steering_noise, distance_noise, sonar_noise, gps_noise, speed, turning_speed,
             gps_delay, execution_cpu_time_limit):

        self.distance_noise = distance_noise
        self.speed = speed
        self.turning_speed = turning_speed
        self.command_queue = []
        self.last_distance = 0.0
        self.steering_noise = steering_noise
        self.gps_noise = gps_noise
        self.gps_delay = gps_delay

        self.measurements = [0.,0.]
        self.move_one = 1.0 / TICK_MOVE
        self.dt = self.move_one * TICK_MOVE / speed

        self.phase = KRC.STATE_DECIDE_NEXT
        self.phase_helper = 0

        self.actual_x = starting_position[0]
        self.actual_y = starting_position[1]
        self.actual_angle = 0

        self.mapa = {}
        self.mapa[(self.actual_x, self.actual_y)] = 1

        #macierze i wektory kalmana

        self.X = matrix([[starting_position[0]], [starting_position[1]], [distance_noise], [distance_noise]])
        self.u = matrix([[0.], [0.], [0.], [0.]])
        self.F = matrix([[1., 0., self.dt, 0.], [0, 1., 0., self.dt], [0., 0., 1., 0.], [0., 0., 0., 1.]])
        self.H = matrix([[1., 0., 0., 0.], [0., 1., 0., 0.]])
        self.P = matrix(
            [[0., 0., 0., 0.], [0., 0., 0., 0.], [0., 0., distance_noise, 0.], [0., 0., 0., distance_noise]])
        self.R = matrix([[gps_noise, 0.0], [0.0, gps_noise]])
        self.I = matrix([[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., 0.], [0., 0., 0., 1.]])

        self.commands = [KRC._DecideNext(self)]
        self.command = KRC._EmptyCommand(self)


    def filter(self, x, P):

        x = (self.F * x) + self.u
        P = self.F * P * self.F.transpose()

        print "poczatek"
        Z = matrix([[self.measurements[0]],[self.measurements[1]]])
        print "utworzono Z"
        y = Z - (self.H * x)
        print "utworzono Y"
        S = self.H * P * self.H.transpose() + self.R
        print "utworzono S"
        K = P * self.H.transpose() * S.inverse()
        print "utworzono K"
        x = x + (K * y)

        P = (self.I - (K * self.H)) * P



        #result
        print 'x= '
        x.show()
        self.X = x
        print 'P= '
        P.show()
        self.P = P




    # def act(self):
    #     print("act")
    #     if len(self.command_queue) == 0:
    #
    #         if self.phase == KRC.STATE_MOTION:
    #             pass
    #         elif self.phase == KRC.STATE_MEASURMENTS:
    #             pass
    #         elif self.phase == MAP_GOAL:
    #             self.command_queue.append([FINISH])
    #
    #         elif self.phase == KRC.STATE_DECIDE_NEXT:
    #             print("decide")
    #             self.mapa[(self.actual_x, self.actual_y)] = 1
    #             neigh = [(self.actual_x + 1, self.actual_y), (self.actual_x, self.actual_y - 1),
    #                      (self.actual_x - 1, self.actual_y), (self.actual_x, self.actual_y + 1)]
    #             if not all((x in self.mapa for x in neigh)):
    #                 self.command_queue.append([SENSE_SONAR])
    #                 self.phase = KRC.STATE_SCANNING
    #                 print("tu skanuje")
    #             else:
    #                 print("elo jestem tu!")
    #                 neigh = [(self.actual_x + 1, self.actual_y), (self.actual_x, self.actual_y - 1),
    #                      (self.actual_x - 1, self.actual_y), (self.actual_x, self.actual_y + 1)]
    #                 find_min = [(self.mapa[x], x) for x in neigh]
    #                 find_min.sort()
    #                 print "Result of scanning, or known before: ", find_min
    #
    #                 goal = find_min[0][1] # Get minimum goal
    #                 print "Goal is ", goal
    #                 vector = (goal[0] - self.actual_x, goal[1] - self.actual_y)
    #                 print "Vector is ", vector
    #                 # Angle calculation + change of coordinates
    #                 angle = math.atan2(-vector[0], vector[1]) / math.pi * 180.0 + 90.0
    #
    #
    #
    #
    #                 print "Current angle is ",self.actual_angle, "rotation by ",(angle - self.angle), " to ",angle
    #                 rotation_ticks = int((angle - self.actual_angle)/180.0 * math.pi / TICK_ROTATE)
    #                 if rotation_ticks != 0 : self.command_queue.append([TURN, rotation_ticks])
    #                 self.actual_angle = angle
    #                 self.command_queue.append([MOVE, int(1.0/TICK_MOVE)])
    #                 self.phase = KRC.STATE_MEASURMENTS
    #
    #             self.state_helper = 0
    #         elif self.phase == KRC.STATE_SCANNING:
    #             print("scanniing")
    #             # Directional cosinus
    #             # Change of coordinaes for convenience
    #             vector = (math.cos((self.actual_angle - 90.0) / 180.0 * math.pi),
    #                       math.sin((self.actual_angle - 90.0) / 180.0 * math.pi))
    #             scanned = (self.actual_x - round(vector[1]), self.actual_y + round(vector[0]))
    #
    #             if (self.actual_x - round(vector[1] + 0.01), self.actual_y + round(vector[0] + 0.01)) not in self.mapa:
    #                 if self.last_distance < 0.9:
    #                     # Strange indexing because x runs vertically and y runs horizontally
    #                     # Set big number so that it won't be visited
    #                     self.mapa[(self.actual_x - round(vector[1] + 0.01), self.actual_y + round(vector[0] + 0.01))] = 1000
    #                 else:
    #                     self.mapa[(self.actual_x - round(vector[1] + 0.01), self.actual_y + round(vector[0] + 0.01))] = 0
    #
    #             self.phase_helper += 1
    #
    #
    #             # It means that we have reached the last rotation
    #             if self.phase_helper == 4:
    #                 self.phase = KRC.STATE_DECIDE_NEXT
    #                 print("tu powinno wrocic")
    #                 print(len(self.command_queue))#tu trzeba cos dodac bo on chce pobrac a nie ma z czego
    #             else:
    #                 self.command_queue.append([TURN, int(0.5 * math.pi / TICK_ROTATE)])
    #                 self.actual_angle = (self.actual_angle + 90.0) % 360
    #                 self.command_queue.append([SENSE_SONAR])
    #
    #     return self.command_queue.pop(0)


    def act(self):
            # run next command from list
            act = None
            while act is None:
                last_result = self.command.done()
                if last_result is not None:
                    if len(self.commands) != 0:
                        self.command = self.commands.pop(0)
                    else:
                        self.command = None

                act = self.command.act()

            return act

    # def on_sense_sonar(self, dist):
    #     self.last_dstance = dist
    #
    # def on_sense_gps(self, x, y):
    #     self.x = x
    #     self.y = y
    #     self.measurements = [x, y]
    #     filter(self, self.X, self.P)
    #
    #


    def on_sense_sonar(self, dist):
        self.last_sonar_read = dist

    def on_sense_field(self, field_type, field_parameter):
        self.last_field_read = field_type, field_parameter

    def on_sense_gps(self, x, y):
        self.last_gps_read = x, y

    class _DecideNext(object):
        def __init__(self, controller):
            self.controller = controller

        def act(self):
            return None

        def done(self):
            controller = self.controller
            #controller.mapa[(controller.actual_x, controller.actual_y)] = 1
            neigh = [(controller.actual_x + 1, controller.actual_y), (controller.actual_x, controller.actual_y - 1),
                     (controller.actual_x - 1, controller.actual_y), (controller.actual_x, controller.actual_y + 1)]
            if not all((x in controller.mapa for x in neigh)):
                controller.commands.append(KRC._ReadSonar(controller))
                controller.commands.append(KRC._StateScanning(controller))
                #controller.phase = KRC.STATE_SCANNING
                print"---przejscie do StateScanning---"
            else:
                print"---decyduje co dalej---"
                neigh = [(controller.actual_x + 1, controller.actual_y), (controller.actual_x, controller.actual_y - 1),
                     (controller.actual_x - 1, controller.actual_y), (controller.actual_x, controller.actual_y + 1)]
                find_min = [(controller.mapa[x], x) for x in neigh]
                find_min.sort()
                print "Result of scanning, or known before: ", find_min

                goal = find_min[0][1] # Get minimum goal
                print "Goal is ", goal
                vector = (goal[0] - controller.actual_x, goal[1] - controller.actual_y)
                print "Vector is ", vector
                # Angle calculation + change of coordinates
                angle = math.atan2(-vector[0], vector[1]) / math.pi * 180.0 + 90.0
                print "Current angle is ",controller.actual_angle, "rotation by ",(angle - controller.actual_angle), " to ",angle
                rotation_ticks = int((angle - controller.actual_angle)/180.0 * math.pi / TICK_ROTATE)
                print(rotation_ticks)
                #if rotation_ticks != 0:
                    #controller.commands.append(KRC._TickTurn(controller,rotation_ticks))
                controller.actual_angle = angle
                print"---aktulny kat: ",controller.actual_angle,"---"
                controller.commands.append(KRC._MoveFwd(controller,vector))
                controller.commands.append(KRC._FindPosition(controller))


            return True

    class _StateScanning(object):
        def __init__(self, controller):
            self.controller = controller
            self.licznik = 0

        def act(self):
            controller = self.controller
            print"--skanuje na pozycji: ", controller.actual_x, " ", controller.actual_y,"---"
            # Directional cosinus
            # Change of coordinaes for convenience
            vector = (math.cos((controller.actual_angle - 90.0) / 180.0 * math.pi),
                      math.sin((controller.actual_angle - 90.0) / 180.0 * math.pi))
            scanned = (controller.actual_x - round(vector[1]), controller.actual_y + round(vector[0]))
            print"--scanned: ",scanned,"--"
            print"--dystans: ",controller.last_distance,"--"
            if (controller.actual_x - round(vector[1] + 0.01), controller.actual_y + round(vector[0] + 0.01)) not in controller.mapa:
                if controller.last_distance < 0.9:
                    # Strange indexing because x runs vertically and y runs horizontally
                    # Set big number so that it won't be visited
                    controller.mapa[(controller.actual_x - round(vector[1] + 0.01), controller.actual_y + round(vector[0] + 0.01))] = 1000
                else:
                    controller.mapa[(controller.actual_x - round(vector[1] + 0.01), controller.actual_y + round(vector[0] + 0.01))] = 0
            self.licznik += 1

            # It means that we have reached the last rotation
            if self.licznik == 4:

                controller.commands.append(KRC._DecideNext(controller))
            else:
                controller.commands.append(KRC._Turn90(controller))
                controller.commands.append(KRC._ReadSonar(controller))
                print"---kat po obrocie: ", controller.actual_angle,"---"
                controller.commands.append(self)
        def done(self):
            return True

    class _FindPosition(object):
        def __init__(self, controller):
            self.controller = controller
            self.helper = 0

        def act(self):
            controller = self.controller
            controller.commands.append(KRC._ReadGPS(controller))
            controller.filter(controller.X, controller.P)

            controller.actual_x = controller.X[0]
            controller.actual_x = int(controller.actual_x[0])
            controller.actual_y = controller.X[1]
            controller.actual_y = int(controller.actual_y[0])
            controller.commands.append(KRC._DecideNext(controller))


        def done(self):
            print"dotarlem do done"
            return True

    class _ReadSonar(object):
        def __init__(self, controller):
            self.controller = controller

        def act(self):
            return [SENSE_SONAR]

        def done(self):
            controller = self.controller
            controller.last_distance = controller.last_sonar_read
            return True

    class _ReadGPS(object):
        def __init__(self,controller):
            self.controller = controller

        def act(self):
            return [SENSE_GPS]

        def done(self):
            controller = self.controller
            controller.measurements[0] = controller.last_gps_read[0]
            controller.measurements[1] = controller.last_gps_read[1]
            return True

    class _Turn90(object):
        def __init__(self, controller):
            self.controller = controller

        def act(self):
            return [TURN, int(0.5 * math.pi / TICK_ROTATE)]

        def done(self):
            controller = self.controller
            controller.actual_angle = (controller.actual_angle + 90.0) % 360
            return True
    class _TickTurn(object):
        def __init__(self, controller, ticks):
            self.controller = controller
            self.ticks = ticks

        def act(self):
            return [TURN, self.ticks]

        def done(self):
            return True

    class _MoveFwd(object):
        def __init__(self, controller, vector):
            self.controller = controller
            self.vector = vector

        def act(self):
            return [MOVE, int(1.0/TICK_MOVE)]

        def done(self):
            controller = self.controller
            # if controller.actual_angle == 0:
            #     controller.actual_y -=1
            # elif controller.actual_angle == 90:
            #     controller.actual_x += 1
            # elif controller.actual_angle == 180:
            #     controller.actual_y += 1
            # elif controller.actual_angle == 270:
            #     controller.actual_x -= 1
            #controller.X[0] = controller.actual_x + self.vector[0]
            #controller.X[1] = controller.actual_y + self.vector[1]
            controller.X = matrix([[controller.actual_x + self.vector[0]],[controller.actual_y + self.vector[1]],[controller.steering_noise],[controller.steering_noise]])
            #controller.commands.append(KRC._DecideNext(controller))
            return True

    class _CheckFieldCommand(object):
        def __init__(self, controller):
            self.controller = controller

        def act(self):
            return [SENSE_FIELD]

        def done(self):
            self.controller.current_field = self.controller.last_field_read
            return self.controller.last_field_read

    class _CheckGoal(object):
        def __init__(self, controller):
            self.controller = controller

        def act(self):
            return [FINISH] if self.controller.current_field[0] == MAP_GOAL else None

        def done(self):
            return True

    class _EmptyCommand(object):
        def __init__(self, *args):
            pass

        def done(self):
            return True

        def act(self):
            return None
