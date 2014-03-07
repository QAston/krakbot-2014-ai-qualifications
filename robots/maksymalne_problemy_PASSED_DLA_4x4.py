from defines import *
from robot_controller import RobotController
import math
import random


class OmitCollisions(RobotController):
    STATE_FORWARD = 0
    STATE_LOOK_FOR_SPACE = 1

    def init(self, starting_position, steering_noise, distance_noise, sonar_noise, measurement_noise, speed, turning_speed,gps_delay,execution_cpu_time_limit):

        self.distance_noise = distance_noise
        self.phase = OmitCollisions.STATE_LOOK_FOR_SPACE
        self.speed = speed
        self.steering_noise = steering_noise
        self.sonar_noise = sonar_noise
        self.turn_speed = turning_speed
        self.command_queue = []
        self.last_distance = 0.0

        if self.sonar_noise != 0:
            self.ile_odczytow_wymagane = self.num_samples_needed(0.99, 0.174, self.sonar_noise)
        else:
            self.ile_odczytow_wymagane = self.num_samples_needed(0.99, 0.174, 0.001)

        self.strona_jazdy = 0



    def act(self):
        if len(self.command_queue) == 0:
            if self.phase == OmitCollisions.STATE_LOOK_FOR_SPACE:
                if self.steering_noise > 0.7:
                    self.command_queue.append([TURN, int(1)])
                else:
                    if self.strona_jazdy == 0:
                        self.command_queue.append([TURN, int(190)])
                    else:
                        self.command_queue.append([TURN, int(-124)])
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

        odczyty = self.ile_odczytow_wymagane
        if odczyty < 1:
            odczyty = 5
        suma = 0
        ruch = 0
        fast_recount = 0.2+self.distance_noise*2.8
        self.command_queue.append([WRITE_CONSOLE, "Num: "+str(odczyty)])
        
        while zlicz < odczyty:
            if distance > 0:
                suma=suma+distance
            zlicz=zlicz+1
            ruch = suma/odczyty

        if ruch < fast_recount:
            self.phase = OmitCollisions.STATE_LOOK_FOR_SPACE
        else:
            if self.strona_jazdy == 1:
                self.strona_jazdy=0
            else:
                self.strona_jazdy=1
            self.command_queue.append([MOVE, (suma/odczyty)-(fast_recount-0.001)])
            self.phase = OmitCollisions.STATE_FORWARD

    def on_sense_field(self, field_type, field_parameter):
        if field_type == MAP_GOAL:
            self.phase = MAP_GOAL

    def num_samples_needed(self,required_certainty, error_margin, noise):
        return int((ltqnorm((required_certainty + 1.0) / 2) * noise / error_margin) ** 2 + 0.5)


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


