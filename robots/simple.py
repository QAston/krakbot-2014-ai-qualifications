from defines import *
from robot_controller import RobotController

class SimpleRobotController(RobotController):
    """
        Failure conditions:
            *  Simulation time has exceeded the given maximum
            *  The robot has exceeded the CPU time limit
            *  The robot has exceeded the RAM limit (note: not controlled in your version)
            *  The robot has reached the goal
            *  The robot has thrown an exception
            *  The robot has exceeded the maximum number of collisions (dependent on the map, but always more than 100)

        Simulator params:
        @param steering_noise - variance of steering in move
        @param distance_noise - variance of distance in move
        @param measurement_noise - variance of GPS measurement
        @param map - map for the robot simulator representing the maze or file to map
        @param init_position - starting position of the Robot (can be moved to map class) [x,y,heading]

        @param speed - distance travelled by one move action (cannot be bigger than 0.5, or he could traverse the walls)
        @param simulation_time_limit - limit in ms for whole robot execution (also with init)
        @param collision_threshold - maximum number of collisions after which robot is destroyed
        @param simulation_dt -  controlls simulation calculation intensivity

        @param frame_dt - save frame every dt
        @param robot - RobotController class that will be simulated in run procedure
        """

    def init(self, starting_position, steering_noise, distance_noise, sonar_noise, gps_noise, speed, turning_speed,gps_delay,execution_cpu_time_limit):
        """
            Specifications of the arguments:
            * starting_position : tuple [x,y,angle], where x and y are accurate positions of the robot (we assume
                upper-left corner is (0,0) and x runs vertically, whereas y runs horizontally) and angle which is an angle in radians
                with respect to X axis
            * steering_noise : sigma of gaussian noise applied to turning motion
            * distance_noise : sigma of gaussian noise applied to forward motion
            * sonar_noise : sigma of gaussian noise applied to sonar
            * gps_noise : sigma of gaussian noise applied to gps measurements
            * speed : speed of the robot in units/simulation_second (speed of the forward motion)
            * turning_speed: turning speed of the robot in radians/simulation_second
            * gps_delay : amount of simulation seconds consumed by a gps measurement
            * execution_cpu_time_limit: total real running time that can be consumed by the robot in seconds

            Bounds:
            * steering_noise : [0, 1.0]
            * sonar_noise : [0, 1.0]
            * gps_noise : [0, 50.0]
            * distance_noise : [0, 1.0]
            * simulation_time_limit : arbitrary
            * speed : [0, 10]
            * turning_speed : [0, 10]
            * gps_delay : [0, 10.0]
            """
        pass

    def act(self):
        """"
            * this is the basic function. It is called repeatedly after the previous command has been executed.
                In act you should return a list. For constants see *defines.py*

            *  Moving : ["move", number_of_ticks] - moves by number_of_ticks*TICK_MOVE in current direction; consumes variable amount of time: number_of_ticks*TICK_MOVE / speed
            *  Turning : ["turn", number_of_ticks] - turns by number_of_ticks*TICK_TURN; consumes variable amount of time: number_of_ticks*TICK_TURN / speed
            *  Sense GPS: ["sense_gps"] - consumes variable amount of time: gps_delay - has gps_noise gaussian noise
            *  Sense sonar: ["sense_sonar"] - consumes constant amount of time : 0.01 simulation time unit - has sonar_noise gaussian noise
            *  Sense field: ["sense_field"] - consumes constant amount of time : 0.01 simulation time_unit - doesn't have read noise
            *  Communicate finish: ["finish"] - consumes 0 units of time
            *  Write to console: ["write_console", string] - consumes 0 units of time, however cannot be used in submission (that is
            code that you submit to us cannot execute action "write_console", you can use it only for debugging)
            """
        return MOVE, 1


    def on_sense_sonar(self, dist):
        pass

    def on_sense_field(self, file_type, file_parameter):
        pass

    def on_sense_gps(self, x, y):
        pass
