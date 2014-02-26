#
# This robot assumes that movement is perfect
# uses distance sensor to see obstacles
# doesn't use GPS
#
from defines import *
from robot_controller import RobotController
import math
import random

class PRC(RobotController):

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

        self.current_position = starting_position[0], starting_position[1]
        self.current_angle = starting_position[2] # in radians
        self.current_field = MAP_START_POSITION, 0
        self.distance_to_obstacle = None

        self.map = {}
        self.times_visited = {}
        self.commands = [self._CheckCurrent(self)] # first command to run
        self.command = PRC._EmptyCommand(self)
        self.map[self.current_position] = self.current_field

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
        return int(self.current_position[0]+0.5) , int(self.current_position[1]+0.5)

    def get_forward_position(self):
        pos = self.get_discrete_position()

        return get_front(pos, self.current_angle)

    def get_neighbour_positions(self):
        #""" returns [front, left, back, right] """"
        curr = self.get_discrete_position()
        ret = []
        angle = self.current_angle;
        ret.append(get_front(curr, angle))
        for i in range(3):
            angle += math.pi * 0.5
            angle %= 2*math.pi
            ret.append(get_front(curr, angle))
        return ret

    def __str__(self):
        return "PCR[pos:{} angl:{}]".format(self.current_position, self.current_angle)


    # commands
    # subclasses of PRC because of the way simulator loads classes
    # act - returns action to do for a robot, can be None
    # done - returns None if action isn't finished yet, other things when finished

    # select what field we should go next to
    class _SelectNext(object):
        def __init__(self, controller):
            self.controller = controller

        def act(self):
            print "Select Next:{}".format(str(self.controller))
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
                print controller.current_position
                print neighbours
                left, back, right = (neighbours[i+1] not in controller.times_visited for i in range(3))

                # choose optimal rotations
                if left and right:
                    c.append(PRC._TurnAngleCommand(controller, 0.5 *math.pi))
                    c.append(PRC._CheckDistanceCommand(self.controller))
                    c.append(PRC._MarkNotVisitedField(self.controller))
                    c.append(PRC._TurnAngleCommand(controller, 0.5 *math.pi))
                    if (back):
                        c.append(PRC._CheckDistanceCommand(self.controller))
                        c.append(PRC._MarkNotVisitedField(self.controller))
                    c.append(PRC._TurnAngleCommand(controller, 0.5 *math.pi))
                    c.append(PRC._CheckDistanceCommand(self.controller))
                    c.append(PRC._MarkNotVisitedField(self.controller))
                elif left:
                    c.append(PRC._TurnAngleCommand(controller, 0.5 *math.pi))
                    c.append(PRC._CheckDistanceCommand(self.controller))
                    c.append(PRC._MarkNotVisitedField(self.controller))
                elif right or back:
                    c.append(PRC._TurnAngleCommand(controller, -0.5 *math.pi))
                    if right:
                        c.append(PRC._CheckDistanceCommand(self.controller))
                        c.append(PRC._MarkNotVisitedField(self.controller))
                    if back:
                        c.append(PRC._TurnAngleCommand(controller, -0.5 *math.pi))
                        c.append(PRC._CheckDistanceCommand(self.controller))
                        c.append(PRC._MarkNotVisitedField(self.controller))
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
                c.append(PRC._CheckDistanceCommand(self.controller))
                c.append(PRC._MarkNotVisitedField(self.controller))

            c.append(PRC._SelectNext(self.controller))
            return None

        def done(self):
            return True

    class _MarkNotVisitedField(object):
        def __init__(self, controller):
            self.controller = controller

        def act(self):
            controller = self.controller

            forward = controller.get_forward_position()


            # mark as wall
            if controller.distance_to_obstacle < 0.9:
                controller.times_visited[forward] = 65536
            # mark as not visited
            else:
                controller.times_visited[forward] = 0

        def done(self):
            return True


    # caution: TICK_MOVE may be not precise enough
    class _MoveDistanceCommand(object):
        def __init__(self, controller, distance):
            self.distance = distance
            self.controller = controller

        def act(self):
            return [MOVE, float(self.distance)/TICK_MOVE]

        def done(self):
            controller = self.controller
            pos = controller.current_position
            orientation = controller.current_angle
            controller.current_position = pos[0] + self.distance * math.cos(orientation), pos[1] + self.distance * math.sin(orientation)
            return True

    # caution: TICK_ROTATE may be not precise enough
    class _TurnAngleCommand(object):
        def __init__(self, controller, angle):
            self.angle = angle
            self.controller = controller

        def act(self):
            print "turning {} angle by: {}".format(str(self.controller), self.angle)
            return [TURN, int(self.angle / TICK_ROTATE)]

        def done(self):
            self.controller.current_angle = (self.controller.current_angle + self.angle) % (2*math.pi)
            return True

    class _CheckDistanceCommand(object):
        def __init__(self, controller):
            self.controller = controller

        def act(self):
            return [SENSE_SONAR]

        def done(self):
            self.controller.distance_to_obstacle = self.controller.last_sonar_read
            return self.controller.last_sonar_read

    class _CheckFieldCommand(object):
        def __init__(self, controller):
            self.controller = controller

        def act(self):
            return [SENSE_FIELD]

        def done(self):
            self.controller.current_field = self.controller.last_field_read
            return self.controller.last_field_read

    class _CheckPositionCommand(object):
        def __init__(self, controller):
            self.controller = controller

        def act(self):
            return [SENSE_GPS]

        def done(self):
            self.controller.current_position = self.controller.last_gps_read
            return self.controller.last_gps_read

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
