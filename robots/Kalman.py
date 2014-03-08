from numpy.matlib import matrix
from defines import *
from robot_controller import RobotController
import math
import random


class KalmanFilter(RobotController):
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

        self.measurements = []
        self.move_one = 1.0 / TICK_MOVE
        self.dt = self.move_one * TICK_MOVE / speed

        self.phase = KalmanFilter.STATE_DECIDE_NEXT
        self.phase_helper = 0

        self.actual_x = starting_position[0]
        self.actual_y = starting_position[1]
        self.actual_angle = starting_position[2]

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


    def filter(self, x, P):

        Z = matrix([[self.measurements]])
        y = Z - (self.H * x)
        S = self.H * P * self.H.transpose() + self.R
        K = P * self.H.transpose() * S.inverse()
        X = x + (K * y)

        P = (self.I - (K * self.H)) * P

        # prediction
        x = (self.F * x) + self.u
        P = self.F * P * self.F.transpose()

        #result
        self.X = x
        self.P = P


    def act(self):
        print("act")
        if len(self.command_queue) == 0:

            if self.phase == KalmanFilter.STATE_MOTION:
                pass
            elif self.phase == KalmanFilter.STATE_MEASURMENTS:
                pass
            elif self.phase == MAP_GOAL:
                self.command_queue.append([FINISH])

            elif self.phase == KalmanFilter.STATE_DECIDE_NEXT:
                print("decide")
                self.mapa[(self.actual_x, self.actual_y)] = 1
                neigh = [(self.actual_x + 1, self.actual_y), (self.actual_x, self.actual_y - 1),
                         (self.actual_x - 1, self.actual_y), (self.actual_x, self.actual_y + 1)]
                if not all((x in self.mapa for x in neigh)):
                    self.command_queue.append([SENSE_SONAR])
                    self.phase = KalmanFilter.STATE_SCANNING
                    print("tu skanuje")
                else:
                    print("elo jestem tu!")
                    neigh = [(self.actual_x + 1, self.actual_y), (self.actual_x, self.actual_y - 1),
                         (self.actual_x - 1, self.actual_y), (self.actual_x, self.actual_y + 1)]
                    find_min = [(self.mapa[x], x) for x in neigh]
                    find_min.sort()
                    print "Result of scanning, or known before: ", find_min

                    goal = find_min[0][1] # Get minimum goal
                    print "Goal is ", goal
                    vector = (goal[0] - self.actual_x, goal[1] - self.actual_y)
                    print "Vector is ", vector
                    # Angle calculation + change of coordinates
                    angle = math.atan2(-vector[0], vector[1]) / math.pi * 180.0 + 90.0




                    print "Current angle is ",self.actual_angle, "rotation by ",(angle - self.angle), " to ",angle
                    rotation_ticks = int((angle - self.actual_angle)/180.0 * math.pi / TICK_ROTATE)
                    if rotation_ticks != 0 : self.command_queue.append([TURN, rotation_ticks])
                    self.actual_angle = angle
                    self.command_queue.append([MOVE, int(1.0/TICK_MOVE)])
                    self.phase = KalmanFilter.STATE_MEASURMENTS

                self.state_helper = 0
            elif self.phase == KalmanFilter.STATE_SCANNING:
                print("scanniing")
                # Directional cosinus
                # Change of coordinaes for convenience
                vector = (math.cos((self.actual_angle - 90.0) / 180.0 * math.pi),
                          math.sin((self.actual_angle - 90.0) / 180.0 * math.pi))
                scanned = (self.actual_x - round(vector[1]), self.actual_y + round(vector[0]))

                if (self.actual_x - round(vector[1] + 0.01), self.actual_y + round(vector[0] + 0.01)) not in self.mapa:
                    if self.last_distance < 0.9:
                        # Strange indexing because x runs vertically and y runs horizontally
                        # Set big number so that it won't be visited
                        self.mapa[(self.actual_x - round(vector[1] + 0.01), self.actual_y + round(vector[0] + 0.01))] = 1000
                    else:
                        self.mapa[(self.actual_x - round(vector[1] + 0.01), self.actual_y + round(vector[0] + 0.01))] = 0

                self.phase_helper += 1


                # It means that we have reached the last rotation
                if self.phase_helper == 4:
                    self.phase = KalmanFilter.STATE_DECIDE_NEXT
                    print("tu powinno wrocic")
                    print(len(self.command_queue))#tu trzeba cos dodac bo on chce pobrac a nie ma z czego
                else:
                    self.command_queue.append([TURN, int(0.5 * math.pi / TICK_ROTATE)])
                    self.actual_angle = (self.actual_angle + 90.0) % 360
                    self.command_queue.append([SENSE_SONAR])

        return self.command_queue.pop(0)


    def on_sense_sonar(self, dist):
        self.last_dstance = dist

    def on_sense_gps(self, x, y):
        self.x = x
        self.y = y
        self.measurements = [x, y]
        filter(self, self.X, self.P)


    def on_sense_field(self, field_type, field_parameter):

        if field_type == MAP_GOAL:
            self.command_queue = [[FINISH]]
