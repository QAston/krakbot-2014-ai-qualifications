from defines import *
from robot_controller import RobotController

class FinisherRobotController(RobotController):
    def init(self, starting_position, steering_noise, distance_noise, sonar_noise, measurement_noise, speed, turning_speed,gps_delay,execution_cpu_time_limit):
        self.command_queue = []

    def act(self):
        if len(self.command_queue) == 0:
            self.command_queue.append([SENSE_FIELD])
            self.command_queue.append([MOVE, 1])

        return self.command_queue.pop(0)


    def on_sense_sonar(self, dist):
        pass

    def on_sense_field(self, field_type, field_parameter):
        if field_type == MAP_GOAL:
            self.command_queue = [[FINISH]]
        pass

    def on_sense_gps(self, x, y):
        pass
