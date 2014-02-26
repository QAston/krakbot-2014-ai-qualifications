#
# This robot assumes that everything is perfect - movement, sensors,speed of gps
#
from defines import *
from robot_controller import RobotController
import math

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
        self.commands = []#[PRC._StateDecideNext(self)]
        self.command = self._EmptyCommand(self)

    def act(self):
        # decide what to do



        act = None
        while act is None:
            last_result = self.command.done()
            if last_result is not None:
                if len(self.commands) != 0:
                    self.command = self.commands.pop(0)
                else:
                    self.commands.append(self._CheckDistanceCommand(self))
                    self.commands.append(self._MoveDistanceCommand(self, 1.0))
                    self.commands.append(self._TurnAngleCommand(self, math.pi*0.5))
                    self.commands.append(self._CheckDistanceCommand(self))
                    self.commands.append(self._CheckFieldCommand(self))
                    self.commands.append(self._CheckGoal(self))

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

    # class _MultiCommand(object):
    #     def __init__(self):
    #         self.commands = []
    #         self.command = PRC._EmptyCommand(None)

    #     def act(self):
    #         # decide what to do
    #         act = None
    #         while act is None:
    #             last_result = self.command.done()
    #             if last_result is not None:
    #                 if len(self.commands) != 0:
    #                     self.command = self.commands.pop(0)
    #                 else:
    #                     self.plan()

    #                     self.command = self.commands.pop(0)
    #             act = self.command.act()

    #         return act

    #     def plan(self):
    #         pass

    #     def done(self):
    #         pass

    # class _State(_MultiCommand):
    #     def __init__(self):
    #         PRC._MultiCommand.__init__(self)
    #         self.planned = False

    #     def plan(self):
    #         if not self.planned:
    #             self.planned = True
    #             self.actions()
    #         else:
    #             self.commands.append(PRC._EmptyCommand(None))

    #     def actions(self):
    #         pass

    #     def done(self):
    #         return True if self.planned and len(self.commands) == 0 else None

    # class _StateDecideNext(_State):
    #     def __init__(self, controller):
    #         PRC._State.__init__(self)
    #         self.controller = controller

    #     def actions(self):
    #         self.commands.append(PRC._CheckFieldCommand(self.controller))
    #         self.commands.append(PRC._CheckGoal(self.controller))
    #         self.commands.append(PRC._ChangeStateCommand(self.controller, PRC._StateMoveNext(self.controller)))


    # class _StateMoveNext(_State):
    #     def __init__(self, controller):
    #         PRC._State.__init__(self)
    #         self.controller = controller

    #     def actions(self):
    #         self.commands.append(PRC._MoveDistanceCommand(self.controller, 1.0))
    #         self.commands.append(PRC._ChangeStateCommand(self.controller, PRC._StateDecideNext(self.controller)))

    # class _ChangeStateCommand(object):
    #     def __init__(self, controller, state):
    #         self.controller = controller
    #         self.state = state

    #     def act(self):
    #         return None

    #     def done(self):
    #         self.controller.commands.append(self.state);
    #         return True


    # class _CommandSequence(_MultiCommand):
    #     def __init__(commands):
    #         self.commands = commands
    #         self.finished = False

    #     def done(self):
    #         return True if self.finished else None

    #     def plan(self):
    #         self.finished = True


    # commands
    # act - returns action to do for a robot, can be None
    # done - returns None if action isn't finished yet, other things when finished

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
            return [TURN, int(angle / TICK_ROTATE)]

        def done(self):
            self.controller.current_angle = (self.controller.current_angle + angle) % (2*math.pi)
            return True

    class _CheckDistanceCommand(object):
        def __init__(self, controller):
            self.controller = controller.last_sonar_read

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
