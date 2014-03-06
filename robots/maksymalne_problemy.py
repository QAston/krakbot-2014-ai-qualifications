from defines import *
from robot_controller import RobotController
import random
from math import pi

class OmitCollisions(RobotController):
    STATE_FORWARD = 0
    STATE_LOOK_FOR_SPACE = 1

    def init(self, starting_position, steering_noise, distance_noise, sonar_noise, measurement_noise, speed, turning_speed,gps_delay,execution_cpu_time_limit):

        self.distance_noise = distance_noise
        self.phase = OmitCollisions.STATE_LOOK_FOR_SPACE
        self.speed = speed
        self.turn_speed = turning_speed
        self.command_queue = []
        self.last_distance = 0.0

    def act(self):
        if len(self.command_queue) == 0:
            if self.phase == OmitCollisions.STATE_LOOK_FOR_SPACE:
                self.command_queue.append([TURN, int((1) / TICK_ROTATE )])
                self.command_queue.append([SENSE_SONAR])
            elif self.phase == MAP_GOAL:
                self.command_queue.append([FINISH])
            else:
                
                self.command_queue.append([SENSE_SONAR])
                self.command_queue.append([SENSE_FIELD])

        return self.command_queue.pop(0)

    def on_sense_sonar(self, distance):
        self.last_distance = distance
        zlicz = 0
        odczyty = 145
        suma = 0
        ruch = 0
        #self.command_queue.append([WRITE_CONSOLE, "Przeszkoda: "+str(distance)])
        
        while zlicz < odczyty:
            if distance > 0:
                #self.command_queue.append([WRITE_CONSOLE, "Dis: "+str(distance)])
                suma=suma+distance
            zlicz=zlicz+1

        ruch = suma/odczyty;

        if ruch < 3:
            self.phase = OmitCollisions.STATE_LOOK_FOR_SPACE
        else:
            self.command_queue.append([MOVE, (suma/odczyty)-2.999])
            self.phase = OmitCollisions.STATE_FORWARD

    def on_sense_field(self, field_type, field_parameter):
        if field_type == MAP_GOAL:
            self.phase = MAP_GOAL

