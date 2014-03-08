#
# This robot assumes that movement is perfect
# uses distance sensor to see obstacles - can work with max noise
# optionally uses gps when driving/steering fails
#
#
from defines import *
from robot_controller import RobotController
import math
import random

import numpy as np
from numpy import matlib
from scipy import stats


class PRC(RobotController):
    DISTANCE_PRECISION_LOW = 2
    DISTANCE_PRECISION_MEDIUM = 1
    DISTANCE_CERTAINTY = 0.9999

    POSITION_CERTAINTY = 0.99

    ANGLE_GPS_CALIBRATE_DIST = 0.2

    MIN_POSITION_PRECISION = 0.2  # how much from theoretical position we can be for algorithm to work
    MAX_ALLOWED_DRIVE_DIFF = 0.3

    def init(self, starting_position, steering_noise, distance_noise, sonar_noise, gps_noise, speed, turning_speed,
             gps_delay, execution_cpu_time_limit):
        self.starting_position = starting_position
        self.steering_noise = steering_noise
        self.distance_noise = distance_noise
        self.sonar_noise = sonar_noise
        self.gps_noise = gps_noise
        self.speed = speed
        self.turning_speed = turning_speed
        self.gps_delay = gps_delay
        self.execution_cpu_time_limit = execution_cpu_time_limit

        # values expected to be approximately correct - should not be used while moving
        self.theoretical_position = starting_position[0], starting_position[1]
        self.theoretical_angle = starting_position[2]  # in radians

        self.current_field = MAP_START_POSITION, 0

        self.map = {}
        self.times_visited = {}

        self.map[self.theoretical_position] = self.current_field

        # the most correct position values we can get
        self.movement_position = self.theoretical_position
        self.movement_angle = self.theoretical_angle

        #scipy doesn't tolerate 0 noise
        if distance_noise > 0:
            self.drive_max_diff = stats.norm.interval(0.98, loc=TICK_MOVE, scale=distance_noise)[1]
        else:
            self.drive_max_diff = TICK_MOVE

        #if self.drive_max_diff - TICK_MOVE >= PRC.MAX_ALLOWED_DRIVE_DIFF:
        #    raise ValueError("This kind of robot won't work with so big diff in movement")

        #CALCULATE SENSOR PRECISIONS

        self.drive_precision = (self.drive_max_diff - TICK_MOVE) / 2
        # calculate how much precision we need from sensors
        self.update_position_precision = PRC.MIN_POSITION_PRECISION - self.drive_precision
        #if self.update_position_precision < 0.0001:
        #    raise ValueError("Cannot get enough precision for algorithm to work")

        self.distance_samples_position = num_samples_needed(PRC.POSITION_CERTAINTY, self.update_position_precision,
                                                            sonar_noise) + 1
        self.gps_samples_position = num_samples_needed(PRC.POSITION_CERTAINTY, self.update_position_precision,
                                                       gps_noise) + 1

        self.has_perfect_steering = steering_noise <= 0.000002

        if steering_noise > 0.0:
            self.steer_max_diff = stats.norm.interval(0.98, loc=TICK_ROTATE, scale=steering_noise)[1]
        else:
            self.steer_max_diff = TICK_ROTATE

        self.steer_precision = self.steer_max_diff - TICK_ROTATE

        # todo: verify constants
        self.gps_samples_calibrate_angle = num_samples_needed(PRC.POSITION_CERTAINTY, 0.01, gps_noise) + 1
        # todo: verify constants - calibration is expensive on gps, so it may be better to skip it if gps takes too long
        self.enable_gps_angle_calibration = steering_noise > 0.0001

        print "self.enable_gps_angle_calibration:{}".format(self.enable_gps_angle_calibration)

        self.angle_dirty = False
        self.angle_change_position = self.movement_position
        self.movement_angle_estimate_updated = False

        # check if we need position corrections
        if self.drive_max_diff - TICK_MOVE <= 0.0003 and steering_noise <= 0.000003:  # just tiny steering changes
            self.update_movement_position_class = PRC._EmptyCommand
            self.update_position_precision = 0.0
        else:
            # gps better than sonar
            if (not self.has_perfect_steering or
                            self.gps_samples_position * self.get_gps_time() <= self.distance_samples_position * self.get_sonar_time()):

                #todo: it may be better to skip gps in some cases because reading it may be expensive
                self.update_movement_position_class = PRC._UpdateMovementPositionWithGps
                # update with real gps precision if better
                if gps_noise == 0.0:
                    self.update_position_precision = 0.0
                else:
                    self.update_position_precision = min(self.update_position_precision,
                                                         stats.norm.interval(PRC.POSITION_CERTAINTY, loc=0.0,
                                                                             scale=gps_noise)[1])

            # sonar better than gps
            else:
                self.update_movement_position_class = PRC._UpdateMovementPositionWithSonar
                # update with real sonar precision if better
                if gps_noise == 0.0:
                    self.update_position_precision = 0.0
                else:
                    self.update_position_precision = min(self.update_position_precision,
                                                         stats.norm.interval(PRC.POSITION_CERTAINTY, loc=0.0,
                                                                             scale=sonar_noise)[1])

        self.detect_wall_precision = 0.2 + PRC.MIN_POSITION_PRECISION - self.update_position_precision

        #distance checks
        self.distance_samples_low = num_samples_needed(PRC.DISTANCE_CERTAINTY, PRC.DISTANCE_PRECISION_LOW,
                                                       sonar_noise) + 1
        self.distance_samples_medium = num_samples_needed(PRC.DISTANCE_CERTAINTY, PRC.DISTANCE_PRECISION_MEDIUM,
                                                          sonar_noise) + 1
        self.distance_samples_detect_wall = num_samples_needed(PRC.DISTANCE_CERTAINTY, self.detect_wall_precision,
                                                               sonar_noise) + 1

        self.gps_cache = []
        self.sonar_cache = []

        print "Info: drive_diff: {}, update_position_precision: {}, detect_wall_precision {}, drive_precision {}".format(
            self.drive_max_diff,
            self.update_position_precision, self.detect_wall_precision, self.drive_precision)

        #TODO: turn to right angle at the beginning of the ride because we start at starting_position[2] which may not be multiplier of Pi/2

        if steering_noise > 0.3 or distance_noise > 0.3:
            self.commands = [self._KKRety(self)] #Uruchomienie kretyna
        else:
            self.commands = [self._CheckCurrent(self)]  # first command to run
        self.command = PRC._EmptyCommand(self)

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

    # sensor readings

    def on_sense_sonar(self, dist):
        self.last_sonar_read = dist

    def on_sense_field(self, field_type, field_parameter):
        self.last_field_read = field_type, field_parameter

    def on_sense_gps(self, x, y):
        self.last_gps_read = x, y

    def get_discrete_position(self):
        return int(self.theoretical_position[0] + 0.5), int(self.theoretical_position[1] + 0.5)

    def get_forward_position(self):
        pos = self.get_discrete_position()

        return get_front(pos, self.theoretical_angle)

    def get_neighbour_positions(self):
        #""" returns [front, left, back, right] """"
        curr = self.get_discrete_position()
        ret = []
        angle = self.theoretical_angle
        ret.append(get_front(curr, angle))
        for i in range(3):
            angle += math.pi * 0.5
            angle %= 2 * math.pi
            ret.append(get_front(curr, angle))
        return ret

    def get_forward_vector(self):
        pos = self.get_discrete_position()
        front = get_front(pos, self.theoretical_angle)
        return front[0] - pos[0], front[1] - pos[1]

    def get_pos_precision(self):
        return self.update_position_precision + self.drive_precision

    def get_sonar_time(self):
        return SONAR_TIME

    def get_gps_time(self):
        return self.gps_delay

    def clear_position_cache(self):
        self.gps_cache[:] = []
        self.sonar_cache[:] = []

    def clear_angle_cache(self):
        self.sonar_cache[:] = []
        if self.enable_gps_angle_calibration:
            self.angle_dirty = True
        self.angle_change_position = self.movement_position
        self.movement_angle_estimate_updated = False

    def get_num_fields_to_next_wall(self):
        curr = self.get_forward_position()
        angle = self.theoretical_angle
        i = 0
        while self.times_visited[curr] < 65536:
            i += 1
            curr = get_front(curr, angle)
        return i

    def __str__(self):
        return "PCR[pos:{} angle:{}]".format(self.theoretical_position, self.theoretical_angle)


    # commands
    # fields of PRC because of the way simulator loads classes
    # act - returns action to do for a robot, can be None
    # done - returns None if action isn't finished yet, other things when finished

    # select what field we should go next to
    class _SelectNext(object):
        def __init__(self, controller):
            self.controller = controller

        def act(self):
            #print "Select Next:{}".format(str(self.controller))
            controller = self.controller
            c = controller.commands

            x_disc, y_disc = controller.get_discrete_position()

            # mark field as known
            controller.map[(x_disc, y_disc)] = controller.current_field

            forward = controller.get_forward_position()

            if controller.times_visited[forward] == 0:
                c.append(PRC._MoveNext(controller))
            else:

                neighbours = controller.get_neighbour_positions()
                front, left, back, right = (neighbours[i] not in controller.times_visited for i in range(4))

                # choose optimal rotations
                if left and right:
                    c.append(PRC._TurnAngleCommand(controller, 0.5 * math.pi))
                    c.append(PRC._CheckWall(self.controller))
                    c.append(PRC._TurnAngleCommand(controller, 0.5 * math.pi))
                    if back:
                        c.append(PRC._CheckWall(self.controller))
                    c.append(PRC._TurnAngleCommand(controller, 0.5 * math.pi))
                    c.append(PRC._CheckWall(self.controller))
                elif back:
                    if left:
                        c.append(PRC._TurnAngleCommand(controller, 0.5 * math.pi))
                        c.append(PRC._CheckWall(self.controller))
                        c.append(PRC._TurnAngleCommand(controller, 0.5 * math.pi))
                        c.append(PRC._CheckWall(self.controller))
                    else:
                        c.append(PRC._TurnAngleCommand(controller, -0.5 * math.pi))
                        if right:
                            c.append(PRC._CheckWall(self.controller))
                        c.append(PRC._TurnAngleCommand(controller, -0.5 * math.pi))
                        c.append(PRC._CheckWall(self.controller))
                elif left:
                    c.append(PRC._TurnAngleCommand(controller, 0.5 * math.pi))
                    c.append(PRC._CheckWall(self.controller))
                elif right:
                    c.append(PRC._TurnAngleCommand(controller, -0.5 * math.pi))
                    c.append(PRC._CheckWall(self.controller))
                else:
                    visits = [controller.times_visited[neigh] for neigh in neighbours]

                    min_visits = min(visits)
                    if min_visits == controller.times_visited[neighbours[0]]:
                        pass
                    elif min_visits == controller.times_visited[neighbours[1]]:
                        c.append(PRC._TurnAngleCommand(controller, 0.5 * math.pi))
                    elif min_visits == controller.times_visited[neighbours[3]]:
                        c.append(PRC._TurnAngleCommand(controller, -0.5 * math.pi))
                    else:
                        c.append(PRC._TurnAngleCommand(controller, math.pi))

                    c.append(PRC._MoveNext(controller))
                    return None

                # try selecting direction once again
                c.append(PRC._SelectNext(controller))
                return None

        def done(self):
            return True

    class _MoveNext(object):
        def __init__(self, controller):
            self.controller = controller

        def act(self):
            c = self.controller.commands
            c.append(PRC._MoveDistanceCommand(self.controller, 1.0))
            c.append(PRC._CheckCurrent(self.controller))
            return None

        def done(self):
            return True

    # caution: precision is controller.drive_max_diff
    class _MoveDistanceCommand(object):
        def __init__(self, controller, distance):
            self.distance = distance
            self.controller = controller
            self.init = False
            self.target_position = None

        def act(self):
            controller = self.controller
            if not self.init:
                self.init = True
                pos = controller.theoretical_position
                orientation = controller.theoretical_angle
                self.target_position = pos[0] + self.distance * math.cos(orientation), pos[
                                                                                           1] + self.distance * math.sin(
                    orientation)
                print "moving {} by: {} to {}".format(str(controller), self.distance, self.target_position)

            vector = (self.target_position[0] - controller.movement_position[0],
                      self.target_position[1] - controller.movement_position[1])

            dist = math.cos(angle_diff(math.atan2(vector[1], vector[0])
                                       , controller.movement_angle)) * vector_length(vector)

            move_times = int(max(dist, 0.0) / controller.drive_max_diff + 0.5)
            if move_times >= 1:
                c = []
                calibrate_move_times = int(PRC.ANGLE_GPS_CALIBRATE_DIST / controller.drive_max_diff + 0.5)
                #only try calibrate if there's enough room
                try_calibrate = dist >= PRC.ANGLE_GPS_CALIBRATE_DIST and controller.angle_dirty
                if controller.movement_angle_estimate_updated:
                    diff = angle_diff(controller.movement_angle, controller.theoretical_angle)
                    turn_times = int(diff / TICK_ROTATE + 0.5)
                    print "correction ticks: {}".format(turn_times)
                    if turn_times != 0:
                        c.append(PRC._TurnAngleTicks(controller, turn_times))
                        move_times = calibrate_move_times
                    else:
                        controller.angle_dirty = False

                if try_calibrate:
                    move_times = calibrate_move_times

                c.append(PRC._MoveTicks(controller, move_times))
                c.append(controller.update_movement_position_class(controller))
                c.append(self)
                controller.commands = c + controller.commands
            else:
                controller.theoretical_position = self.target_position

            return None


        def done(self):
            return True

    #low level function, use other ones instead
    class _MoveTicks(object):
        def __init__(self, controller, move_times):
            self.move_times = move_times
            self.controller = controller

        def act(self):
            controller = self.controller
            #print "MoveTicks {} ticks: {}".format(str(controller), self.move_times)
            #simulate perfect movement, don't account for randomness as result will be different from robot's rand anyways
            controller.movement_position = simulate_move(self.move_times, 0, controller.movement_position,
                                                         controller.movement_angle)

            # account for distribution bias
            controller.movement_position = calculate_law_of_big_numbers_error(self.move_times,
                                                                              controller.distance_noise,
                                                                              controller.movement_position,
                                                                              controller.movement_angle)

            #print "-movement_position: {}".format(controller.movement_position)
            return [MOVE, self.move_times]

        def done(self):
            self.controller.clear_position_cache()
            return True

    class _UpdateMovementPositionWithGps(object):
        def __init__(self, controller):
            self.controller = controller
            self.samples = controller.gps_cache

        def act(self):
            return [SENSE_GPS]

        def done(self):
            controller = self.controller
            self.samples.append(controller.last_gps_read)
            num_samples = len(self.samples)
            if controller.angle_dirty and num_samples >= controller.gps_samples_calibrate_angle:
                sum = 0, 0
                for sample in self.samples:
                    sum = sum[0] + sample[0], sum[1] + sample[1]

                controller.movement_position = sum[0] / num_samples, sum[1] / num_samples
                vector = controller.movement_position[0] - controller.angle_change_position[0], \
                         controller.movement_position[1] - controller.angle_change_position[1]
                if vector_length(vector) >= PRC.ANGLE_GPS_CALIBRATE_DIST:
                    controller.movement_angle = math.atan2(vector[1], vector[0])
                    controller.movement_angle_estimate_updated = True
                return True

            elif (not controller.angle_dirty) and num_samples >= controller.gps_samples_position:

                sum = 0, 0
                for sample in self.samples:
                    sum = sum[0] + sample[0], sum[1] + sample[1]

                controller.movement_position = sum[0] / num_samples, sum[1] / num_samples

                #print "UpdateMPosWithGps: samples {}, move_pos{}".format(num_samples, controller.movement_position)
                return True
            return None

    class _UpdateMovementPositionWithSonar(object):
        def __init__(self, controller):
            self.controller = controller
            self.samples = controller.sonar_cache
            if not controller.has_perfect_steering:
                raise ValueError("self.has_perfect_steering is not set but movement relies on steering")

        def act(self):
            return [SENSE_SONAR]

        def done(self):
            controller = self.controller
            self.samples.append(controller.last_sonar_read)
            num_samples = len(self.samples)

            if num_samples >= controller.distance_samples_position:
                distance = sum(self.samples) / num_samples
                fields_to_wall = controller.get_num_fields_to_next_wall()
                target_dist = fields_to_wall + 0.5

                fwd = controller.get_forward_vector()
                controller.movement_position = (controller.theoretical_position[0] + (target_dist - distance) * fwd[0],
                                                controller.theoretical_position[1] + (target_dist - distance) * fwd[1])

                #print "UpdateMPosWithSonar: target_dist {}, distance {}, move_pos{}".format(target_dist, distance, controller.movement_position)

                return True
            return None


    # called once per visit
    class _CheckCurrent(object):
        def __init__(self, controller):
            self.controller = controller

        def act(self):
            controller = self.controller
            c = controller.commands
            x_disc, y_disc = controller.get_discrete_position()
            # get field type
            if (x_disc, y_disc) not in controller.map:
                c.append(PRC._CheckFieldCommand(self.controller))
                c.append(PRC._CheckGoal(self.controller))
            else:
                controller.current_field = controller.map[(x_disc, y_disc)]

            # mark visited
            if (x_disc, y_disc) not in controller.times_visited:
                controller.times_visited[(x_disc, y_disc)] = 1
            else:
                controller.times_visited[(x_disc, y_disc)] += 1

            forward = controller.get_forward_position()

            if forward not in controller.times_visited:
                c.append(PRC._CheckWall(self.controller))

            c.append(PRC._SelectNext(self.controller))
            return None

        def done(self):
            return True

    # class _CheckHints(object):
    #     def __init__(self, controller):
    #         self.controller = controller

    #     def act(self):
    #         controller = self.controller
    #         c = controller.commands

    #         if controller.current_field[0] == MAP_SPECIAL_OPTIMAL:
    #             c.append(PRC._CheckDistanceCommand(self.controller))
    #             c.append(PRC._CheckDistanceCommand(self.controller))

    #         c.append(PRC._SelectNext(self.controller))
    #         return None

    #     def done(self):
    #         return True

    class _CheckWall(object):
        def __init__(self, controller):
            self.controller = controller
            self.samples = controller.sonar_cache

        def act(self):
            return [SENSE_SONAR]

        def done(self):
            controller = self.controller
            self.samples.append(controller.last_sonar_read)
            num_samples = len(self.samples)
            # from highest to lowest-when equal highest takes precedence
            if num_samples >= controller.distance_samples_detect_wall:
                #print "Check wall:{}".format(str(self.controller))
                #print "-high-samples:{}".format(num_samples)
                distance = sum(self.samples) / num_samples
                #print "-distance:{}".format(distance)
                distance -= 0.5
                fields = int(distance + 0.5)
                #print "-fields:{}".format(fields)
                controller.mark_clear_fields(fields, True)
                return True

            # for perfect steering always use precise dist check
            elif (not controller.has_perfect_steering
                  and (
                                num_samples >= controller.distance_samples_medium or num_samples >= controller.distance_samples_low)):
                #print "Check wall:{}".format(str(self.controller))
                #print "-med, low-samples:{}".format(num_samples)
                distance = sum(self.samples) / num_samples
                #print "-distance:{}".format(distance)
                fields = controller.get_clear_fields(distance,
                                                     PRC.DISTANCE_PRECISION_LOW if num_samples == controller.distance_samples_low else PRC.DISTANCE_PRECISION_MEDIUM)
                #print "-fields:{}".format(fields)
                if fields > 1:
                    controller.mark_clear_fields(fields, False)
                    return True

            return None

    def get_clear_fields(self, distance, precision):
        distance -= 0.5 + self.get_pos_precision() + precision
        return int(math.floor(distance))

    def mark_clear_fields(self, dist, mark_wall):
        # mark fields clear only for near fields, but when self.has_perfect_steering mark all fields up to the wall
        distance = dist if self.has_perfect_steering else min(5, dist)
        cur = self.get_discrete_position()
        for i in range(distance):
            cur = get_front(cur, self.theoretical_angle)
            if cur not in self.times_visited:
                self.times_visited[cur] = 0
        if (mark_wall and dist == distance) or self.has_perfect_steering:
            cur = get_front(cur, self.theoretical_angle)
            self.times_visited[cur] = 65536

    # caution: TICK_ROTATE may be not precise enough
    class _TurnAngleCommand(object):
        def __init__(self, controller, angle):
            self.angle = angle
            self.controller = controller
            self.target_angle = None
            self.init = False
            self.turn_angle_cmd = None

        def act(self):
            controller = self.controller
            if not self.init:
                self.init = True
                self.target_angle = (controller.theoretical_angle + self.angle) % (2 * math.pi)

            change = angle_diff(controller.movement_angle, self.target_angle)
            #print "turning {} angle by: {}".format(str(controller), self.angle)
            turn_times = int(change / TICK_ROTATE + 0.5)
            self.turn_angle_cmd = PRC._TurnAngleTicks(controller, turn_times)
            return self.turn_angle_cmd.act()

        def done(self):
            self.turn_angle_cmd.done()
            controller = self.controller
            controller.theoretical_angle = self.target_angle
            #print "-theoretical_angle {}".format(controller.theoretical_angle)
            return True

    class _TurnAngleTicks(object):
        def __init__(self, controller, turn_times):
            self.controller = controller
            self.turn_times = turn_times

        def act(self):
            controller = self.controller
            #simulate perfect movement, don't account for randomness as result will be different from robot's rand anyways
            controller.movement_angle = simulate_turn(self.turn_times, 0, controller.movement_angle)
            return [TURN, self.turn_times]

        def done(self):
            self.controller.clear_angle_cache()
            #print "movement_angle {}".format(controller.movement_angle)
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

    class _KKRety(object):
        def __init__(self, controller):
            self.controller = controller
            self.phase = 0
            self.samples = controller.sonar_cache
            self.odczyty = num_samples_needed(0.99, 0.4, controller.sonar_noise)
            if self.odczyty < 1:
                self.odczyty = 5

        def act(self):
            return [SENSE_SONAR]

        def done(self):
            controller = self.controller
            c = controller.commands
            c.append(PRC._CheckFieldCommand(self.controller))
            c.append(PRC._CheckGoal(self.controller))
            self.samples.append(controller.last_sonar_read)

            print self.samples[-1]
            print len(self.samples)
            print self.odczyty

            if len(self.samples) < self.odczyty:
                return None
            else:
                ruch = sum(self.samples)/self.odczyty
                controller.clear_angle_cache()

                print "D:"+str(controller.drive_max_diff)
                print "R:"+str(ruch)

                if ruch < 0.4:
                    c.append(controller._TurnAngleTicks(controller,int(1)))
                else:
                    #if controller.drive_max_diff > ruch:
                        c.append(controller._KK_MOVE(controller, (ruch-0.3999)))
                c.append(self)
                return True

            return True

        def sumPl(self,w):
            s = 0.0
            for x in w:
                if x > 0.0:
                    s += x
            return s

    class _KK_MOVE(object):
        def __init__(self, controller, move_var):
            self.move_var = move_var
            self.controller = controller

        def act(self):
            controller = self.controller
            return [MOVE, self.move_var]

        def done(self):
            return True

def get_front(pos, angle):
    dist = 1.0
    return int(pos[0] + 0.5 + dist * math.cos(angle)), int(pos[1] + 0.5 + dist * math.sin(angle))


# simulate turning as it happens exactly in the guts of simulator
# damn floating point
def simulate_turn(turn_times, steering_noise, orientation):
    number_steps = abs(turn_times)
    step = 1 if turn_times == number_steps else -1
    for i in xrange(number_steps):
        turn = random.gauss(step * TICK_ROTATE, steering_noise)
        orientation = (orientation + turn) % (2 * math.pi)
    return orientation


def simulate_move(move_times, distance_noise, position, orientation):
    cos_ort = math.cos(orientation)
    sin_ort = math.sin(orientation)
    for i in xrange(move_times):
        distance = max(0.0, random.gauss(TICK_MOVE, distance_noise))
        position = position[0] + distance * cos_ort, position[1] + distance * sin_ort
    return position


def calculate_law_of_big_numbers_error(move_times, distance_noise, position, orientation):
    cos_ort = math.cos(orientation)
    sin_ort = math.sin(orientation)
    for i in xrange(move_times):
        # add negative error to simulator position, so in error calc it shows up positive
        distance = -min(0.0, random.gauss(TICK_MOVE, distance_noise))
        position = position[0] + distance * cos_ort, position[1] + distance * sin_ort
    return position


# calculates signed diff between 2 angles
def angle_diff(x, y):
    return min(y - x, y - x + 2 * math.pi, y - x - 2 * math.pi, key=abs)


def vector_length(v):
    return math.sqrt(v[0] * v[0] + v[1] * v[1])


# returns number of samples needed to estimate value to a certain certainty
def num_samples_needed(required_certainty, error_margin, noise):
    return int((ltqnorm((required_certainty + 1.0) / 2) * noise / error_margin) ** 2 + 0.5)

def kalman_measurement_update(state, uncertainty, measurement, measurement_function, measurement_uncertainty, i):
    z = measurement.transpose()
    y = z - measurement_function*state
    s = measurement_function*uncertainty*measurement_function.transpose() + measurement_uncertainty
    k = uncertainty*measurement_function.transpose()*s.inverse()
    state = state+k*y
    uncertainty = (i-k*measurement_function)*uncertainty

def kalman_prediction(state, uncertainty, next_state_function, external_state_change):
    state = next_state_function*state+external_state_change
    uncertainty=next_state_function*uncertainty*next_state_function.transpose()
    return state, uncertainty


#inverse of cumulative function of normal distribution
def ltqnorm(p):
    """
    Modified from the author's original perl code (original comments follow below)
    by dfield@yahoo-inc.com.  May 3, 2004.

    Lower tail quantile for standard normal distribution function.

    This function returns an approximation of the inverse cumulative
    standard normal distribution function.  I.e., given P, it returns
    an approximation to the X satisfying P = Pr{Z <= X} where Z is a
    random variable from the standard normal distribution.

    The algorithm uses a minimax approximation by rational functions
    and the result has a relative error whose absolute value is less
    than 1.15e-9.

    Author:      Peter John Acklam
    Time-stamp:  2000-07-19 18:26:14
    E-mail:      pjacklam@online.no
    WWW URL:     http://home.online.no/~pjacklam
    """

    if p <= 0 or p >= 1:
        # The original perl code exits here, we'll throw an exception instead
        raise ValueError("Argument to ltqnorm %f must be in open interval (0,1)" % p)

    # Coefficients in rational approximations.
    a = (-3.969683028665376e+01, 2.209460984245205e+02,
         -2.759285104469687e+02, 1.383577518672690e+02,
         -3.066479806614716e+01, 2.506628277459239e+00)
    b = (-5.447609879822406e+01, 1.615858368580409e+02,
         -1.556989798598866e+02, 6.680131188771972e+01,
         -1.328068155288572e+01 )
    c = (-7.784894002430293e-03, -3.223964580411365e-01,
         -2.400758277161838e+00, -2.549732539343734e+00,
         4.374664141464968e+00, 2.938163982698783e+00)
    d = ( 7.784695709041462e-03, 3.224671290700398e-01,
          2.445134137142996e+00, 3.754408661907416e+00)

    # Define break-points.
    plow = 0.02425
    phigh = 1 - plow

    # Rational approximation for lower region:
    if p < plow:
        q = math.sqrt(-2 * math.log(p))
        return (((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]) / \
               ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1)

    # Rational approximation for upper region:
    if phigh < p:
        q = math.sqrt(-2 * math.log(1 - p))
        return -(((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]) / \
               ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1)

    # Rational approximation for central region:
    q = p - 0.5
    r = q * q
    return (((((a[0] * r + a[1]) * r + a[2]) * r + a[3]) * r + a[4]) * r + a[5]) * q / \
           (((((b[0] * r + b[1]) * r + b[2]) * r + b[3]) * r + b[4]) * r + 1)
