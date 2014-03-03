#
# This robot assumes that movement is perfect
# uses distance sensor to see obstacles - can work with max noise
# optionally uses gps when driving/steering fails
#
#todo: optimize this code:P
#todo: compare avg speeds with other algorithms to choose the fastest for given config
#
from defines import *
from robot_controller import RobotController
import math
import random

from scipy import stats;

class PRC(RobotController):

    DISTANCE_PRECISION_LOW = 2
    DISTANCE_PRECISION_MEDIUM = 1
    DISTANCE_CERTAINTY = 0.9999

    POSITION_CERTAINTY = 0.99

    MIN_POSITION_PRECISION = 0.2 # how much from theoretical position we can be for alorithm to work
    MAX_ALLOWED_DRIVE_DIFF = 0.3

    def init(self, starting_position, steering_noise, distance_noise, sonar_noise, gps_noise, speed, turning_speed,gps_delay,execution_cpu_time_limit):
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
        self.theoretical_angle = starting_position[2] # in radians

        self.current_field = MAP_START_POSITION, 0
        self.distance_to_obstacle = None

        self.map = {}
        self.times_visited = {}
        self.commands = [self._CheckCurrent(self)] # first command to run
        self.command = PRC._EmptyCommand(self)
        self.map[self.theoretical_position] = self.current_field

        # the most correct position values we can get
        self.movement_position = self.theoretical_position
        self.movement_angle = self.theoretical_angle

        #scipy doesn't tolerate 0 noise
        if distance_noise > 0:
            self.drive_max_diff = stats.norm.interval(0.98, loc=TICK_MOVE, scale=distance_noise)[1]
        else:
            self.drive_max_diff = TICK_MOVE

        if self.drive_max_diff - TICK_MOVE >= PRC.MAX_ALLOWED_DRIVE_DIFF:
            raise ValueError("This kind of robot won't work with so big diff in movement")

        self.angle_error = 0.0

        #CALCULATE SENSOR PRECISIONS

        self.drive_precision = (self.drive_max_diff - TICK_MOVE)  / 2
        # calculate how much precision we need from sensors
        self.update_position_precision = PRC.MIN_POSITION_PRECISION - self.drive_precision
        if self.update_position_precision < 0.0001:
            raise ValueError("Cannot get enough precision for algorithm to work")

        self.distance_samples_position = num_samples_needed(PRC.POSITION_CERTAINTY, self.update_position_precision, sonar_noise) + 1
        self.gps_samples_position = num_samples_needed(PRC.POSITION_CERTAINTY, self.update_position_precision, gps_noise) + 1

        #TODO: verify this constant!
        # check if we need position corrections
        if self.drive_max_diff < 0.05:
            self.update_movement_position_class = PRC._EmptyCommand
            self.update_position_precision = 0.0
        else:
            #todo: prefer gps for big steering_noise
            # gps better than sonar
            if self.gps_samples_position * self.get_gps_time() <= self.distance_samples_position * self.get_sonar_time():
                self.update_movement_position_class = PRC._UpdateMovementPositionWithGps
                # update with real gps precision if better
                if gps_noise == 0.0:
                    self.update_position_precision = 0.0
                else:
                    self.update_position_precision = min(self.update_position_precision, stats.norm.interval(PRC.POSITION_CERTAINTY, loc=0.0, scale=gps_noise)[1])

            # sonar better than gps
            else:
                self.update_movement_position_class = PRC._UpdateMovementPositionWithSonar
                # update with real sonar precision if better
                if gps_noise == 0.0:
                    self.update_position_precision = 0.0
                else:
                    self.update_position_precision = min(self.update_position_precision, stats.norm.interval(PRC.POSITION_CERTAINTY, loc=0.0, scale=sonar_noise)[1])

        self.detect_wall_precision = 0.2 + PRC.MIN_POSITION_PRECISION - self.update_position_precision

        #distance checks
        self.distance_samples_low = num_samples_needed(PRC.DISTANCE_CERTAINTY, PRC.DISTANCE_PRECISION_LOW, sonar_noise) + 1
        self.distance_samples_medium = num_samples_needed(PRC.DISTANCE_CERTAINTY, PRC.DISTANCE_PRECISION_MEDIUM, sonar_noise) + 1
        self.distance_samples_detect_wall = num_samples_needed(PRC.DISTANCE_CERTAINTY, self.detect_wall_precision, sonar_noise) + 1


        print "Info: drive_diff: {}, update_position_precision: {}, detect_wall_precision {}, drive_precision {}".format(self.drive_max_diff,
            self.update_position_precision, self.detect_wall_precision, self.drive_precision)

        #TODO: turn to right angle at the beginning of the ride becase we start at starting_position[2] which may not be multiplier of Pi/2

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
        self.last_sonar_read = dist;

    def on_sense_field(self, field_type, field_parameter):
        self.last_field_read = field_type, field_parameter

    def on_sense_gps(self, x, y):
        self.last_gps_read = x, y

    def get_discrete_position(self):
        return int(self.theoretical_position[0]+0.5) , int(self.theoretical_position[1]+0.5)

    def get_forward_position(self):
        pos = self.get_discrete_position()

        return get_front(pos, self.theoretical_angle)

    def get_neighbour_positions(self):
        #""" returns [front, left, back, right] """"
        curr = self.get_discrete_position()
        ret = []
        angle = self.theoretical_angle;
        ret.append(get_front(curr, angle))
        for i in range(3):
            angle += math.pi * 0.5
            angle %= 2*math.pi
            ret.append(get_front(curr, angle))
        return ret

    def get_pos_precision(self):
        return self.update_position_precision + self.drive_precision

    def get_sonar_time(self):
        return SONAR_TIME

    def get_gps_time(self):
        return self.gps_delay

    def __str__(self):
        return "PCR[pos:{} angl:{}]".format(self.theoretical_position, self.theoretical_angle)


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
                    c.append(PRC._TurnAngleCommand(controller, 0.5 *math.pi))
                    c.append(PRC._CheckWall(self.controller))
                    c.append(PRC._TurnAngleCommand(controller, 0.5 *math.pi))
                    if (back):
                        c.append(PRC._CheckWall(self.controller))
                    c.append(PRC._TurnAngleCommand(controller, 0.5 *math.pi))
                    c.append(PRC._CheckWall(self.controller))
                elif back:
                    if left:
                        c.append(PRC._TurnAngleCommand(controller, 0.5 *math.pi))
                        c.append(PRC._CheckWall(self.controller))
                        c.append(PRC._TurnAngleCommand(controller, 0.5 *math.pi))
                        c.append(PRC._CheckWall(self.controller))
                    else:
                        c.append(PRC._TurnAngleCommand(controller, -0.5 *math.pi))
                        if right:
                            c.append(PRC._CheckWall(self.controller))
                        c.append(PRC._TurnAngleCommand(controller, -0.5 *math.pi))
                        c.append(PRC._CheckWall(self.controller))
                elif left:
                    c.append(PRC._TurnAngleCommand(controller, 0.5 *math.pi))
                    c.append(PRC._CheckWall(self.controller))
                elif right:
                    c.append(PRC._TurnAngleCommand(controller, -0.5 *math.pi))
                    c.append(PRC._CheckWall(self.controller))
                else:
                    visits = [controller.times_visited[neigh] for neigh in neighbours]

                    min_visits = min(visits)
                    if min_visits == controller.times_visited[neighbours[0]]:
                        pass
                    elif min_visits == controller.times_visited[neighbours[1]]:
                        c.append(PRC._TurnAngleCommand(controller, 0.5 *math.pi))
                    elif min_visits == controller.times_visited[neighbours[3]]:
                        c.append(PRC._TurnAngleCommand(controller, -0.5 *math.pi))
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
                #print "moving {} by: {} to {}".format(str(controller), self.distance, self.target_position)
                self.init = True
                pos = controller.theoretical_position
                orientation = controller.theoretical_angle
                self.target_position = pos[0] + self.distance * math.cos(orientation), pos[1] + self.distance * math.sin(orientation)
                controller.theoretical_position = self.target_position

            vector = (self.target_position[0] - controller.movement_position[0],
                                             self.target_position[1] - controller.movement_position[1])

            dist = math.cos(angle_diff(math.atan2(vector[1], vector[0])
                , controller.movement_angle)) * vector_length(vector)

            move_times = int(max(dist, 0.0)/controller.drive_max_diff + 0.5)
            if move_times >= 1:
                controller.commands = [PRC._MoveTicks(controller, move_times), controller.update_movement_position_class(controller), self] + controller.commands
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
            controller.movement_position = simulate_move(self.move_times, 0, controller.movement_position, controller.movement_angle)

            # account for distribution bias
            controller.movement_position = calculate_law_of_big_numbers_error(self.move_times,
                controller.distance_noise, controller.movement_position, controller.movement_angle)

            #print "-movement_position: {}".format(controller.movement_position)
            return [MOVE, self.move_times]

        def done(self):
            return True

    class _UpdateMovementPositionWithGps(object):
        def __init__(self, controller):
            self.controller = controller
            self.init = False;
            self.samples = []

        def act(self):
            if not self.init:
                self.init = True;
                #todo, cache gps samples
            return [SENSE_GPS]
        def done(self):
            controller = self.controller
            self.samples.append(controller.last_gps_read)
            num_samples = len(self.samples)
            # from highest to lowest-when equal highest takes precenence
            if (num_samples >= controller.gps_samples_position):

                sum = 0,0
                for sample in self.samples:
                    sum = sum[0] + sample[0], sum[1] + sample[1]

                controller.movement_position = sum[0]/num_samples, sum[1]/num_samples

                #print "UpdateMPosWithGps: samples {}, move_pos{}".format(num_samples, controller.movement_position)
                return True
            return None

    class _UpdateMovementPositionWithSonar(object):
        def __init__(self, controller):
            self.controller = controller
            self.init = False;
            self.samples = []

        def act(self):
            if not self.init:
                self.init = True;
                #todo, cache sonar samples
            return [SENSE_SONAR]

        def done(self):
            controller = self.controller
            self.samples.append(controller.last_sonar_read)
            num_samples = len(self.samples)
            # from highest to lowest-when equal highest takes precenence
            if (num_samples >= controller.distance_samples_position):
                #TODO: calc movement position from sonar
                distance = sum(self.samples)/num_samples
                #print "UpdateMPosWithSonar: samples {}, move_pos{}".format(num_samples, controller.movement_position)

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
            self.samples = []

        def act(self):
            return [SENSE_SONAR]

        def done(self):
            controller = self.controller
            self.samples.append(controller.last_sonar_read)
            num_samples = len(self.samples)
            # from highest to lowest-when equal highest takes precenence
            if (num_samples >= controller.distance_samples_detect_wall):
                print "Check wall:{}".format(str(self.controller))
                print "-high-samples:{}".format(num_samples)
                distance = sum(self.samples)/num_samples
                print "-distance:{}".format(distance)
                distance -= 0.5
                fields = int(distance + 0.5)
                print "-fields:{}".format(fields)
                controller.mark_clear_fields(fields, True)
                return True
            elif (num_samples == controller.distance_samples_medium or num_samples == controller.distance_samples_low):
                print "Check wall:{}".format(str(self.controller))
                print "-med, low-samples:{}".format(num_samples)
                distance = sum(self.samples)/num_samples
                print "-distance:{}".format(distance)
                fields = controller.get_clear_fields(distance,
                            PRC.DISTANCE_PRECISION_LOW if num_samples == controller.distance_samples_low else PRC.DISTANCE_PRECISION_MEDIUM)
                print "-fields:{}".format(fields)
                if fields > 1:
                    controller.mark_clear_fields(fields, False)
                    return True

            return None

    def get_clear_fields(self, distance, precision):
        distance -= 0.5 + self.get_pos_precision() + precision
        return int(math.floor(distance))

    def mark_clear_fields(self, dist, mark_wall):
        distance = min(5, dist) # assume angle inprecision
        cur = self.get_discrete_position()
        for i in range(distance):
            cur = get_front(cur, self.theoretical_angle)
            if cur not in self.times_visited:
                self.times_visited[cur] = 0
        if mark_wall and dist == distance:
            cur = get_front(cur, self.theoretical_angle)
            self.times_visited[cur] = 65536

    # caution: TICK_ROTATE may be not precise enough
    class _TurnAngleCommand(object):
        def __init__(self, controller, angle):
            self.angle = angle
            self.controller = controller

        def act(self):
            controller = self.controller

            #print "turning {} angle by: {}".format(str(controller), self.angle)
            turn_times = int((self.angle + controller.angle_error)/ TICK_ROTATE + 0.5)
            #controller.angle_error = self.angle - TICK_ROTATE * turn_times
            ##print "error: {}".format(controller.angle_error)

            #simulate perfect movement, don't account for randomness as result will be different from robot's rand anyways
            controller.movement_angle = simulate_turn(turn_times, 0, controller.movement_angle)

            return [TURN, turn_times]

        def done(self):
            controller = self.controller
            controller.theoretical_angle = (controller.theoretical_angle + self.angle) % (2*math.pi)

            controller.angle_error = angle_diff(controller.movement_angle, controller.theoretical_angle)
            #print "sim angle {} real angle {} angle error {}".format(controller.movement_angle, controller.theoretical_angle, controller.angle_error)
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
        def __init__(self, controller):
            pass

        def done(self):
            return True

        def act(self):
            return None

def get_front(pos, angle):
    dist = 1.0;
    return int(pos[0] + 0.5 + dist * math.cos(angle)) , int(pos[1] + 0.5 + dist * math.sin(angle))

# simulate turning as it happens exactly in the guts of simulator
# damn floating point
def simulate_turn(turn_times, steering_noise, orientation):
    number_steps = abs(turn_times)
    step = 1 if turn_times == number_steps else -1
    for i in xrange(number_steps):
        turn = random.gauss(step*TICK_ROTATE, steering_noise)
        orientation = (orientation+turn)%(2*math.pi)
    return orientation

def simulate_move(move_times, distance_noise, position, orientation):
    cos_ort = math.cos(orientation)
    sin_ort = math.sin(orientation)
    for i in xrange(move_times):
        distance = max(0.0,random.gauss(TICK_MOVE, distance_noise))
        position = position[0] + distance * cos_ort, position[1] + distance * sin_ort
    return position

def calculate_law_of_big_numbers_error(move_times, distance_noise, position, orientation):
    cos_ort = math.cos(orientation)
    sin_ort = math.sin(orientation)
    for i in xrange(move_times):
        # add negative error to simulator position, so in error calc it shows up positive
        distance = -min(0.0,random.gauss(TICK_MOVE, distance_noise))
        position = position[0] + distance * cos_ort, position[1] + distance * sin_ort
    return position

# calcs signed diff between 2 angles
def angle_diff(x, y):
    return min(y-x, y-x+2*math.pi, y-x-2*math.pi, key=abs)

def vector_length(v):
    return math.sqrt(v[0]*v[0] + v[1] * v[1])

# returns number of samples needed to estimate value to a certain certainty
def num_samples_needed(required_certainty, error_margin, noise):
    return int((ltqnorm((required_certainty + 1.0) / 2) * noise / error_margin) **2 + 0.5)

#inverse of cumulative function of normal distribution
def ltqnorm( p ):
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
        raise ValueError( "Argument to ltqnorm %f must be in open interval (0,1)" % p )

    # Coefficients in rational approximations.
    a = (-3.969683028665376e+01,  2.209460984245205e+02, \
         -2.759285104469687e+02,  1.383577518672690e+02, \
         -3.066479806614716e+01,  2.506628277459239e+00)
    b = (-5.447609879822406e+01,  1.615858368580409e+02, \
         -1.556989798598866e+02,  6.680131188771972e+01, \
         -1.328068155288572e+01 )
    c = (-7.784894002430293e-03, -3.223964580411365e-01, \
         -2.400758277161838e+00, -2.549732539343734e+00, \
          4.374664141464968e+00,  2.938163982698783e+00)
    d = ( 7.784695709041462e-03,  3.224671290700398e-01, \
          2.445134137142996e+00,  3.754408661907416e+00)

    # Define break-points.
    plow  = 0.02425
    phigh = 1 - plow

    # Rational approximation for lower region:
    if p < plow:
       q  = math.sqrt(-2*math.log(p))
       return (((((c[0]*q+c[1])*q+c[2])*q+c[3])*q+c[4])*q+c[5]) / \
               ((((d[0]*q+d[1])*q+d[2])*q+d[3])*q+1)

    # Rational approximation for upper region:
    if phigh < p:
       q  = math.sqrt(-2*math.log(1-p))
       return -(((((c[0]*q+c[1])*q+c[2])*q+c[3])*q+c[4])*q+c[5]) / \
                ((((d[0]*q+d[1])*q+d[2])*q+d[3])*q+1)

    # Rational approximation for central region:
    q = p - 0.5
    r = q*q
    return (((((a[0]*r+a[1])*r+a[2])*r+a[3])*r+a[4])*r+a[5])*q / \
           (((((b[0]*r+b[1])*r+b[2])*r+b[3])*r+b[4])*r+1)
