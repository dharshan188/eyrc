#!/usr/bin/env python3

import json
import math
import time

import paho.mqtt.client as mqtt


# ================================================================
# MQTT
# ================================================================

MQTT_HOST = "localhost"
MQTT_PORT = 1883

TOPIC_SENSORS = "pacbot/sensors"
TOPIC_WHEEL_VEL = "pacbot/wheel_vel"


# ================================================================
# MAIN SPEED
# ================================================================

# Full speed ONLY when stable/centered.
BASE_SPEED = 7.5

# One-wall/open-area speed.
ONE_WALL_SPEED = 6.0

# No useful walls.
OPEN_SPEED = 6.3

# Immediately after turn.
SETTLE_SPEED = 3.8

# Maximum speed during turn lockout.
LOCKOUT_SPEED = 4.5

MAX_WHEEL = 9.0


# ================================================================
# SENSOR FILTER
#
# Filtering is VERY important.
# Raw readings at ~500 Hz can jump.
# ================================================================

SENSOR_ALPHA = 0.10


# ================================================================
# MAZE ENTRY / FIRST TURN
# ================================================================

INSIDE_SIDE_DIST = 0.12

# Preserve first trigger.
FIRST_FRONT_DIST = 0.14

FIRST_FRONT_DEBOUNCE = 4


# ================================================================
# IMPORTANT:
#
# THERE IS NO 90 DEG PHYSICAL SCAN ANYMORE.
#
# MAX path is measured while STOPPED.
#
# Every actual turn:
#
#            TARGET = 80 DEG
# ================================================================

TURN_TARGET = 80.0

TURN_TOLERANCE = 0.55


# ================================================================
# TURN CONTROL
#
# Strong when far.
# Very gentle near 80 degrees.
# ================================================================

TURN_KP = 0.052

TURN_KD = 0.012

TURN_MAX_SPEED = 2.60


# Overshoot correction speed
TURN_REVERSE_MAX = 0.70


# If somehow angle goes this high, force reverse correction.
TURN_RESCUE_ANGLE = 82.0

# Absolute emergency protection.
TURN_HARD_LIMIT = 84.0


TURN_TIMEOUT = 4.5


# ================================================================
# TURN TIMING
# ================================================================

BRAKE_TIME = 0.12

TURN_STOP_TIME = 0.10

SETTLE_TIME = 0.22

POST_TURN_LOCKOUT = 0.60


# ================================================================
# CORNER DETECTION
#
# Do NOT turn because of one random low side reading.
#
# Confirmed corner:
#
# minimum side <= 0.12
# maximum side <= 0.16
# ================================================================

SIDE_TURN_DIST = 0.12

SIDE_CONFIRM_DIST = 0.16

SIDE_ARM_DIST = 0.18

CORNER_CONFIRM_TIME = 0.025


# ================================================================
# MAX PATH SAMPLING
#
# Robot stops for a VERY short time and averages all 4 sensors.
#
# Gyro verifies robot is stable.
# ================================================================

MAX_PATH_SAMPLE_TIME = 0.045

MAX_PATH_SAMPLE_TIMEOUT = 0.14

MAX_PATH_SENSOR_CAP = 1.0

MAX_PATH_GYRO_LIMIT = 0.30


# Front rays matter more for open path.
MAX_PATH_FRONT_WEIGHT = 0.65

MAX_PATH_SIDE_WEIGHT = 0.35


# Need a useful difference before declaring winner.
MAX_PATH_MIN_DIFFERENCE = 0.035


# ================================================================
# WALL FOLLOWING
#
# MOST IMPORTANT STRAIGHT CONTROL:
#
# If both walls exist:
#
#              make SL == SR
#
# We do NOT force both sensors to some arbitrary value.
#
# Equal distance = centered robot.
# ================================================================

WALL_MIN = 0.025

WALL_MAX = 0.35


# Used ONLY when there is one wall.
ONE_WALL_TARGET = 0.15


# ================================================================
# CENTERING PID
#
# Strong P = quickly return to center.
# Small I = prevent slow bias.
# D = prevent oscillation.
# ================================================================

CENTER_KP = 14.0

CENTER_KI = 0.020

CENTER_KD = 0.12


# One-wall control should be gentler.
ONE_WALL_KP = 7.0


CENTER_INTEGRAL_LIMIT = 0.040

CENTER_ERROR_LIMIT = 0.12

CENTER_DEADBAND = 0.0015


DERIVATIVE_ALPHA = 0.08


# ================================================================
# HEADING HOLD
#
# Previous versions only used instantaneous gyro.
#
# Now we INTEGRATE gyro while driving.
#
# Therefore if robot becomes 3-5 degrees crooked,
# it actively returns to the corridor heading.
# ================================================================

HEADING_KP = 0.060

HEADING_RATE_KD = 0.008


# ================================================================
# MAX NORMAL CORRECTION
# ================================================================

MAX_CORRECTION = 3.2


# Normal correction slew.
CORRECTION_SLEW = 28.0


# ================================================================
# FRONT COLLISION SAFETY
#
# User wanted hard stop lower than before.
#
# Warning     0.12
# Strong      0.09
# Hard stop   0.07
#
# IMPORTANT:
#
# A SINGLE FL/FR low reading does NOT stop robot.
# It STEERS AWAY.
#
# BOTH below 0.07 = actual dead-end stop.
# ================================================================

FRONT_WARNING_DIST = 0.12

FRONT_DANGER_DIST = 0.09

FRONT_STOP_DIST = 0.07


FRONT_WARNING_CORRECTION = 0.45

FRONT_DANGER_MIN_CORRECTION = 1.20

FRONT_DANGER_MAX_CORRECTION = 2.80

FRONT_DANGER_GAIN = 36.0


# ================================================================
# SIDE COLLISION SAFETY
#
# Early correction before actual collision.
# ================================================================

SIDE_WARNING_DIST = 0.115

SIDE_DANGER_DIST = 0.085

SIDE_EMERGENCY_DIST = 0.055


SIDE_WARNING_CORRECTION = 0.50

SIDE_DANGER_MIN_CORRECTION = 1.20

SIDE_DANGER_MAX_CORRECTION = 2.80

SIDE_DANGER_GAIN = 34.0


# ================================================================
# PREDICTIVE WALL PROTECTION
#
# Predict wall distance ~55 ms into future.
# ================================================================

PREDICT_HORIZON = 0.055

MAX_CLOSING_RATE = 1.5


# ================================================================
# CONTROLLER
# ================================================================

class Controller:

    def __init__(self):

        # --------------------------------------------------------
        # STATES
        #
        # DRIVE
        # PATH_SAMPLE
        # BRAKE_TURN
        # TURN
        # TURN_STOP
        # SETTLE
        # --------------------------------------------------------

        self.mode = "DRIVE"


        # ========================================================
        # MAZE / TURN
        # ========================================================

        self.inside_maze = False

        self.completed_turns = 0

        self.first_front_count = 0


        # +1 LEFT
        # -1 RIGHT
        self.turn_dir = +1


        # ========================================================
        # FILTERED SENSOR VALUES
        # ========================================================

        self.filters_ready = False

        self.f_sl = 0.0
        self.f_sr = 0.0

        self.f_fl = 0.0
        self.f_fr = 0.0


        # ========================================================
        # CENTER PID
        # ========================================================

        self.center_integral = 0.0

        self.previous_center_error = 0.0

        self.filtered_derivative = 0.0

        self.have_previous_error = False

        self.previous_correction = 0.0


        # ========================================================
        # DRIVE HEADING
        #
        # Integrated yaw error in degrees.
        # ========================================================

        self.drive_yaw_deg = 0.0

        self.drive_gyro_bias = 0.0


        # ========================================================
        # PREDICTIVE SIDE HISTORY
        # ========================================================

        self.prev_sl = None

        self.prev_sr = None


        # ========================================================
        # CORNER
        # ========================================================

        self.side_detector_armed = False

        self.corner_timer = 0.0


        # ========================================================
        # PATH SAMPLE
        # ========================================================

        self.path_sample_time = 0.0

        self.path_total_time = 0.0

        self.path_sample_count = 0


        self.path_fl_sum = 0.0
        self.path_fr_sum = 0.0

        self.path_sl_sum = 0.0
        self.path_sr_sum = 0.0

        self.path_gz_sum = 0.0


        self.path_last_fl = 0.0
        self.path_last_fr = 0.0

        self.path_last_sl = 0.0
        self.path_last_sr = 0.0

        self.path_last_gz = 0.0


        # Was this sampling for first turn?
        self.path_is_first = False


        # ========================================================
        # TURN GYRO
        # ========================================================

        self.gyro_bias = 0.0

        self.gyro_sum = 0.0

        self.gyro_samples = 0


        self.turn_angle = 0.0

        self.turn_time = 0.0


        # ========================================================
        # TIMERS
        # ========================================================

        self.brake_time = 0.0

        self.turn_stop_time = 0.0

        self.settle_time = 0.0


        self.sim_time = 0.0

        self.turn_lockout_until = 0.0


        # ========================================================
        # LOG
        # ========================================================

        self.last_log = 0.0


    # ============================================================
    # BASIC HELPERS
    # ============================================================

    @staticmethod
    def clamp(
        value,
        low,
        high
    ):

        return max(
            low,
            min(
                high,
                value
            )
        )


    @staticmethod
    def clean_sensor(
        value,
        cap
    ):

        if value < 0.0:

            return 0.0


        return min(
            value,
            cap
        )


    @staticmethod
    def wall_visible(value):

        return (
            WALL_MIN
            <
            value
            <
            WALL_MAX
        )


    # ============================================================
    # FILTER SENSORS
    #
    # Cap values only for filtering.
    # Huge no-hit numbers become 1.0.
    # ============================================================

    def update_filters(
        self,
        sl,
        sr,
        fl,
        fr
    ):

        slc = self.clean_sensor(
            sl,
            1.0
        )

        src = self.clean_sensor(
            sr,
            1.0
        )

        flc = self.clean_sensor(
            fl,
            1.0
        )

        frc = self.clean_sensor(
            fr,
            1.0
        )


        if not self.filters_ready:

            self.f_sl = slc
            self.f_sr = src

            self.f_fl = flc
            self.f_fr = frc

            self.filters_ready = True


            return


        a = SENSOR_ALPHA


        self.f_sl = (

            a * slc

            +

            (1.0 - a) * self.f_sl
        )


        self.f_sr = (

            a * src

            +

            (1.0 - a) * self.f_sr
        )


        self.f_fl = (

            a * flc

            +

            (1.0 - a) * self.f_fl
        )


        self.f_fr = (

            a * frc

            +

            (1.0 - a) * self.f_fr
        )


    # ============================================================
    # RESET DRIVE CONTROLLER
    # ============================================================

    def reset_drive_controller(self):

        self.center_integral = 0.0

        self.previous_center_error = 0.0

        self.filtered_derivative = 0.0

        self.have_previous_error = False

        self.previous_correction = 0.0


        self.drive_yaw_deg = 0.0


        self.prev_sl = None

        self.prev_sr = None


    # ============================================================
    # RESET PATH SAMPLES
    # ============================================================

    def reset_path_samples(self):

        self.path_sample_time = 0.0

        self.path_total_time = 0.0

        self.path_sample_count = 0


        self.path_fl_sum = 0.0
        self.path_fr_sum = 0.0

        self.path_sl_sum = 0.0
        self.path_sr_sum = 0.0

        self.path_gz_sum = 0.0


    # ============================================================
    # PREDICTIVE WALL SPEED
    # ============================================================

    def predictive_speed(
        self,
        sl,
        sr,
        dt,
        requested_speed
    ):

        if (
            self.prev_sl is None

            or

            self.prev_sr is None

            or

            dt <= 0.0001
        ):

            self.prev_sl = sl

            self.prev_sr = sr


            return requested_speed


        sl_rate = (

            self.prev_sl
            -
            sl

        ) / dt


        sr_rate = (

            self.prev_sr
            -
            sr

        ) / dt


        self.prev_sl = sl

        self.prev_sr = sr


        # Only closing motion matters.
        sl_rate = self.clamp(
            sl_rate,
            0.0,
            MAX_CLOSING_RATE
        )


        sr_rate = self.clamp(
            sr_rate,
            0.0,
            MAX_CLOSING_RATE
        )


        predicted_sl = (

            sl

            -

            sl_rate
            *
            PREDICT_HORIZON
        )


        predicted_sr = (

            sr

            -

            sr_rate
            *
            PREDICT_HORIZON
        )


        predicted = min(
            predicted_sl,
            predicted_sr
        )


        speed = requested_speed


        if predicted <= 0.065:

            speed = min(
                speed,
                2.0
            )


        elif predicted <= 0.085:

            speed = min(
                speed,
                3.0
            )


        elif predicted <= 0.105:

            speed = min(
                speed,
                4.2
            )


        elif predicted <= 0.130:

            speed = min(
                speed,
                5.6
            )


        return speed


    # ============================================================
    # FRONT SAFETY
    #
    # Positive correction = steer RIGHT
    #
    # Negative correction = steer LEFT
    # ============================================================

    def front_safety(
        self,
        fl,
        fr
    ):

        # ========================================================
        # TRUE DEAD END
        # ========================================================

        if (
            fl <= FRONT_STOP_DIST

            and

            fr <= FRONT_STOP_DIST
        ):

            return (
                0.0,
                True,
                True,
                "BOTH_FRONT_STOP"
            )


        # ========================================================
        # LEFT FRONT DANGER
        #
        # Immediately turn away to RIGHT.
        # ========================================================

        if fl <= FRONT_DANGER_DIST:

            error = (
                FRONT_DANGER_DIST
                -
                fl
            )


            correction = (

                FRONT_DANGER_MIN_CORRECTION

                +

                FRONT_DANGER_GAIN
                *
                error
            )


            correction = self.clamp(
                correction,
                FRONT_DANGER_MIN_CORRECTION,
                FRONT_DANGER_MAX_CORRECTION
            )


            return (
                +correction,
                False,
                True,
                "FL_DANGER_RIGHT"
            )


        # ========================================================
        # RIGHT FRONT DANGER
        # ========================================================

        if fr <= FRONT_DANGER_DIST:

            error = (
                FRONT_DANGER_DIST
                -
                fr
            )


            correction = (

                FRONT_DANGER_MIN_CORRECTION

                +

                FRONT_DANGER_GAIN
                *
                error
            )


            correction = self.clamp(
                correction,
                FRONT_DANGER_MIN_CORRECTION,
                FRONT_DANGER_MAX_CORRECTION
            )


            return (
                -correction,
                False,
                True,
                "FR_DANGER_LEFT"
            )


        # ========================================================
        # LEFT WARNING
        # ========================================================

        if fl <= FRONT_WARNING_DIST:

            return (
                +FRONT_WARNING_CORRECTION,
                False,
                False,
                "FL_WARN_RIGHT"
            )


        # ========================================================
        # RIGHT WARNING
        # ========================================================

        if fr <= FRONT_WARNING_DIST:

            return (
                -FRONT_WARNING_CORRECTION,
                False,
                False,
                "FR_WARN_LEFT"
            )


        return (
            0.0,
            False,
            False,
            "CLEAR"
        )


    # ============================================================
    # SIDE SAFETY
    # ============================================================

    def side_safety(
        self,
        sl,
        sr
    ):

        # ========================================================
        # LEFT DANGER
        # ========================================================

        if (
            sl <= SIDE_DANGER_DIST

            and

            sr > SIDE_DANGER_DIST
        ):

            error = (
                SIDE_DANGER_DIST
                -
                sl
            )


            correction = (

                SIDE_DANGER_MIN_CORRECTION

                +

                SIDE_DANGER_GAIN
                *
                error
            )


            correction = self.clamp(
                correction,
                SIDE_DANGER_MIN_CORRECTION,
                SIDE_DANGER_MAX_CORRECTION
            )


            return (
                +correction,
                True,
                "SL_DANGER_RIGHT"
            )


        # ========================================================
        # RIGHT DANGER
        # ========================================================

        if (
            sr <= SIDE_DANGER_DIST

            and

            sl > SIDE_DANGER_DIST
        ):

            error = (
                SIDE_DANGER_DIST
                -
                sr
            )


            correction = (

                SIDE_DANGER_MIN_CORRECTION

                +

                SIDE_DANGER_GAIN
                *
                error
            )


            correction = self.clamp(
                correction,
                SIDE_DANGER_MIN_CORRECTION,
                SIDE_DANGER_MAX_CORRECTION
            )


            return (
                -correction,
                True,
                "SR_DANGER_LEFT"
            )


        # ========================================================
        # LEFT WARNING
        # ========================================================

        if (
            sl <= SIDE_WARNING_DIST

            and

            sr > SIDE_WARNING_DIST
        ):

            return (
                +SIDE_WARNING_CORRECTION,
                False,
                "SL_WARN_RIGHT"
            )


        # ========================================================
        # RIGHT WARNING
        # ========================================================

        if (
            sr <= SIDE_WARNING_DIST

            and

            sl > SIDE_WARNING_DIST
        ):

            return (
                -SIDE_WARNING_CORRECTION,
                False,
                "SR_WARN_LEFT"
            )


        # Both close usually means confirmed-corner region.
        if (
            sl <= SIDE_DANGER_DIST

            and

            sr <= SIDE_DANGER_DIST
        ):

            return (
                0.0,
                False,
                "BOTH_SIDE_CLOSE"
            )


        return (
            0.0,
            False,
            "CLEAR"
        )


    # ============================================================
    # DETERMINE DRIVE SPEED
    #
    # High speed ONLY when actually centered.
    # ============================================================

    def drive_speed(
        self,
        sl,
        sr,
        lockout_active
    ):

        left_wall = self.wall_visible(
            sl
        )

        right_wall = self.wall_visible(
            sr
        )


        # ========================================================
        # TWO WALLS
        #
        # Difference tells us how centered we are.
        # ========================================================

        if (
            left_wall
            and
            right_wall
        ):

            difference = abs(
                sr
                -
                sl
            )


            if difference <= 0.008:

                speed = BASE_SPEED


            elif difference <= 0.015:

                speed = 6.8


            elif difference <= 0.025:

                speed = 5.8


            elif difference <= 0.040:

                speed = 4.5


            else:

                # Strongly off-center.
                # Stabilize first.
                speed = 3.4


        # ========================================================
        # ONE WALL
        # ========================================================

        elif (
            left_wall
            or
            right_wall
        ):

            speed = ONE_WALL_SPEED


        # ========================================================
        # NO WALL
        # ========================================================

        else:

            speed = OPEN_SPEED


        # ========================================================
        # APPROACHING VERY CLOSE SIDE WALL
        # ========================================================

        nearest = min(
            sl,
            sr
        )


        if nearest <= 0.075:

            speed = min(
                speed,
                2.2
            )


        elif nearest <= 0.095:

            speed = min(
                speed,
                3.3
            )


        elif nearest <= 0.120:

            speed = min(
                speed,
                4.8
            )


        if lockout_active:

            speed = min(
                speed,
                LOCKOUT_SPEED
            )


        return speed


    # ============================================================
    # MAIN STRAIGHT CONTROLLER
    #
    # BOTH WALLS:
    #
    #       maintain SL == SR
    #
    # ONE WALL:
    #
    #       maintain ~0.15 and heading lock
    #
    # ============================================================

    def straight_drive(
        self,
        sl,
        sr,
        fl,
        fr,
        gz,
        dt,
        requested_speed,
        collision_safety=True
    ):

        left_wall = self.wall_visible(
            sl
        )

        right_wall = self.wall_visible(
            sr
        )


        # ========================================================
        # DRIVE YAW INTEGRATION
        #
        # Positive GZ = LEFT rotation.
        #
        # Positive correction drives RIGHT, so positive yaw
        # naturally generates positive correction.
        # ========================================================

        corrected_gz = (
            gz
            -
            self.drive_gyro_bias
        )


        gyro_rate_deg = (

            corrected_gz

            *

            180.0

            /

            math.pi
        )


        self.drive_yaw_deg += (
            gyro_rate_deg
            *
            dt
        )


        # Prevent runaway if gyro is noisy.
        self.drive_yaw_deg = self.clamp(
            self.drive_yaw_deg,
            -15.0,
            15.0
        )


        # ========================================================
        # WALL ERROR
        # ========================================================

        if (
            left_wall
            and
            right_wall
        ):

            # ----------------------------------------------------
            # THIS IS THE MAIN CENTERING RULE.
            #
            # Same distance:
            #
            # sr - sl -> 0
            # ----------------------------------------------------

            error = (
                sr
                -
                sl
            )


            drive_mode = "CENTER"


            wall_kp = CENTER_KP


        elif left_wall:

            # Left too close:
            # target - actual becomes positive -> turn right.
            error = (
                ONE_WALL_TARGET
                -
                sl
            )


            drive_mode = "LEFT"

            wall_kp = ONE_WALL_KP


        elif right_wall:

            # Right too close:
            # actual - target becomes negative -> turn left.
            error = (
                sr
                -
                ONE_WALL_TARGET
            )


            drive_mode = "RIGHT"

            wall_kp = ONE_WALL_KP


        else:

            error = 0.0

            drive_mode = "HEADING"

            wall_kp = 0.0


        # ========================================================
        # ERROR CLEANUP
        # ========================================================

        if abs(error) < CENTER_DEADBAND:

            error = 0.0


        error = self.clamp(
            error,
            -CENTER_ERROR_LIMIT,
            CENTER_ERROR_LIMIT
        )


        # ========================================================
        # INTEGRAL
        # ========================================================

        if (
            left_wall
            and
            right_wall
            and
            abs(error) < 0.035
        ):

            self.center_integral += (
                error
                *
                dt
            )


        else:

            # Quickly clear stale integral.
            self.center_integral *= 0.92


        self.center_integral = self.clamp(
            self.center_integral,
            -CENTER_INTEGRAL_LIMIT,
            CENTER_INTEGRAL_LIMIT
        )


        # ========================================================
        # DERIVATIVE
        # ========================================================

        if (
            self.have_previous_error

            and

            dt > 0.0001
        ):

            raw_derivative = (

                error
                -
                self.previous_center_error

            ) / dt


        else:

            raw_derivative = 0.0


        self.filtered_derivative = (

            DERIVATIVE_ALPHA
            *
            raw_derivative

            +

            (
                1.0
                -
                DERIVATIVE_ALPHA
            )
            *
            self.filtered_derivative
        )


        self.previous_center_error = error

        self.have_previous_error = True


        # ========================================================
        # WALL PID
        # ========================================================

        wall_correction = (

            wall_kp
            *
            error

            +

            CENTER_KI
            *
            self.center_integral

            +

            CENTER_KD
            *
            self.filtered_derivative
        )


        # ========================================================
        # HEADING LOCK
        #
        # Fixes the previous problem:
        #
        # robot could be angled but GZ near zero.
        #
        # Now accumulated yaw still corrects it.
        # ========================================================

        heading_correction = (

            HEADING_KP
            *
            self.drive_yaw_deg

            +

            HEADING_RATE_KD
            *
            gyro_rate_deg
        )


        correction = (

            wall_correction

            +

            heading_correction
        )


        front_mode = "OFF"

        side_mode = "OFF"


        danger_override = False


        # ========================================================
        # SAFETY
        # ========================================================

        if collision_safety:

            (
                front_correction,
                front_stop,
                front_danger,
                front_mode
            ) = self.front_safety(
                fl,
                fr
            )


            (
                side_correction,
                side_danger,
                side_mode
            ) = self.side_safety(
                sl,
                sr
            )


            # ====================================================
            # TRUE DEAD END
            # ====================================================

            if front_stop:

                return (
                    0.0,
                    0.0,
                    drive_mode,
                    error,
                    front_mode,
                    side_mode
                )


            correction += (
                front_correction

                +

                side_correction
            )


            danger_override = (
                front_danger

                or

                side_danger
            )


            # ====================================================
            # DANGER SPEED
            # ====================================================

            if front_danger:

                requested_speed = min(
                    requested_speed,
                    3.0
                )


            elif front_mode != "CLEAR":

                requested_speed = min(
                    requested_speed,
                    5.0
                )


            if side_danger:

                requested_speed = min(
                    requested_speed,
                    3.0
                )


            elif side_mode != "CLEAR":

                requested_speed = min(
                    requested_speed,
                    4.8
                )


        # ========================================================
        # LIMIT CORRECTION
        # ========================================================

        correction = self.clamp(
            correction,
            -MAX_CORRECTION,
            MAX_CORRECTION
        )


        # ========================================================
        # CRITICAL:
        #
        # If already in wall danger, DON'T wait for slow slew.
        #
        # React immediately.
        # ========================================================

        if danger_override:

            final_correction = correction

            self.previous_correction = (
                final_correction
            )


        else:

            max_change = (

                CORRECTION_SLEW
                *
                dt
            )


            final_correction = (

                self.previous_correction

                +

                self.clamp(

                    correction
                    -
                    self.previous_correction,

                    -max_change,

                    max_change
                )
            )


            self.previous_correction = (
                final_correction
            )


        # ========================================================
        # WHEELS
        # ========================================================

        left = (

            requested_speed

            +

            final_correction
        )


        right = (

            requested_speed

            -

            final_correction
        )


        # Never reverse while normal straight driving.
        left = self.clamp(
            left,
            0.0,
            MAX_WHEEL
        )


        right = self.clamp(
            right,
            0.0,
            MAX_WHEEL
        )


        return (
            left,
            right,
            drive_mode,
            error,
            front_mode,
            side_mode
        )


    # ============================================================
    # CORNER CONDITION
    # ============================================================

    @staticmethod
    def corner_condition(
        sl,
        sr
    ):

        nearest = min(
            sl,
            sr
        )


        farther = max(
            sl,
            sr
        )


        return (

            nearest
            <=
            SIDE_TURN_DIST

            and

            farther
            <=
            SIDE_CONFIRM_DIST
        )


    # ============================================================
    # PATH SCORES
    # ============================================================

    def calculate_path_scores(
        self,
        fl,
        fr,
        sl,
        sr
    ):

        flc = self.clean_sensor(
            fl,
            MAX_PATH_SENSOR_CAP
        )


        frc = self.clean_sensor(
            fr,
            MAX_PATH_SENSOR_CAP
        )


        slc = self.clean_sensor(
            sl,
            MAX_PATH_SENSOR_CAP
        )


        src = self.clean_sensor(
            sr,
            MAX_PATH_SENSOR_CAP
        )


        left_score = (

            MAX_PATH_FRONT_WEIGHT
            *
            flc

            +

            MAX_PATH_SIDE_WEIGHT
            *
            slc
        )


        right_score = (

            MAX_PATH_FRONT_WEIGHT
            *
            frc

            +

            MAX_PATH_SIDE_WEIGHT
            *
            src
        )


        return (
            left_score,
            right_score
        )


    # ============================================================
    # MAX PATH CHOICE
    # ============================================================

    def choose_max_path(
        self,
        fl,
        fr,
        sl,
        sr,
        gz
    ):

        flc = self.clean_sensor(
            fl,
            MAX_PATH_SENSOR_CAP
        )


        frc = self.clean_sensor(
            fr,
            MAX_PATH_SENSOR_CAP
        )


        slc = self.clean_sensor(
            sl,
            MAX_PATH_SENSOR_CAP
        )


        src = self.clean_sensor(
            sr,
            MAX_PATH_SENSOR_CAP
        )


        (
            left_score,
            right_score
        ) = self.calculate_path_scores(
            flc,
            frc,
            slc,
            src
        )


        diff = (
            left_score
            -
            right_score
        )


        print()
        print("==================================================")
        print(" MAX PATH DECISION")
        print("==================================================")


        print(
            f"FL={flc:.3f} "
            f"FR={frc:.3f} "
            f"SL={slc:.3f} "
            f"SR={src:.3f}"
        )


        print(
            f"GZ={gz:+.4f}"
        )


        print()


        print(
            f"LEFT  = "
            f"{left_score:.4f}"
        )


        print(
            f"RIGHT = "
            f"{right_score:.4f}"
        )


        # ========================================================
        # LEFT CLEAR WIN
        # ========================================================

        if (
            diff
            >
            MAX_PATH_MIN_DIFFERENCE
        ):

            print(
                "MAX PATH -> LEFT"
            )

            print("==================================================")
            print()


            return (
                +1,
                (
                    f"MAX LEFT "
                    f"{left_score:.3f} > "
                    f"{right_score:.3f}"
                )
            )


        # ========================================================
        # RIGHT CLEAR WIN
        # ========================================================

        if (
            diff
            <
            -MAX_PATH_MIN_DIFFERENCE
        ):

            print(
                "MAX PATH -> RIGHT"
            )

            print("==================================================")
            print()


            return (
                -1,
                (
                    f"MAX RIGHT "
                    f"{right_score:.3f} > "
                    f"{left_score:.3f}"
                )
            )


        # ========================================================
        # CLOSE SCORES
        #
        # Use front difference first.
        # ========================================================

        if flc > frc + 0.02:

            print(
                "CLOSE SCORE -> FL LARGER -> LEFT"
            )

            print("==================================================")
            print()


            return (
                +1,
                "TIE -> FL MORE OPEN -> LEFT"
            )


        if frc > flc + 0.02:

            print(
                "CLOSE SCORE -> FR LARGER -> RIGHT"
            )

            print("==================================================")
            print()


            return (
                -1,
                "TIE -> FR MORE OPEN -> RIGHT"
            )


        # ========================================================
        # FINAL FALLBACK
        #
        # Keep successful inverted side rule.
        #
        # SL closer -> RIGHT
        # SR closer -> LEFT
        # ========================================================

        if slc < src:

            print(
                "FINAL FALLBACK -> RIGHT"
            )

            print("==================================================")
            print()


            return (
                -1,
                "TIE -> SL CLOSER -> RIGHT"
            )


        else:

            print(
                "FINAL FALLBACK -> LEFT"
            )

            print("==================================================")
            print()


            return (
                +1,
                "TIE -> SR CLOSER -> LEFT"
            )


    # ============================================================
    # START PATH SAMPLE
    #
    # No rotating scan.
    #
    # This is why the first turn cannot accidentally go to 90+
    # before trying to align back.
    # ============================================================

    def start_path_sample(
        self,
        sl,
        sr,
        fl,
        fr,
        gz,
        is_first=False
    ):

        self.mode = "PATH_SAMPLE"


        self.path_is_first = is_first


        self.reset_path_samples()


        self.path_last_fl = fl
        self.path_last_fr = fr

        self.path_last_sl = sl
        self.path_last_sr = sr

        self.path_last_gz = gz


        self.reset_drive_controller()


        if is_first:

            print()
            print("==================================================")
            print(" FIRST TURN -> MAX PATH SAMPLE")
            print(" NO 90 DEG SCAN")
            print("==================================================")
            print()


        else:

            print()
            print("==================================================")
            print(" CORNER -> MAX PATH SAMPLE")
            print("==================================================")
            print()


        return (
            0.0,
            0.0
        )


    # ============================================================
    # PATH SAMPLE
    # ============================================================

    def path_sample(
        self,
        sl,
        sr,
        fl,
        fr,
        gz,
        dt
    ):

        self.path_total_time += dt


        self.path_last_fl = fl
        self.path_last_fr = fr

        self.path_last_sl = sl
        self.path_last_sr = sr

        self.path_last_gz = gz


        # ========================================================
        # ONLY STABLE SAMPLES
        # ========================================================

        if abs(gz) <= MAX_PATH_GYRO_LIMIT:

            flc = self.clean_sensor(
                fl,
                MAX_PATH_SENSOR_CAP
            )


            frc = self.clean_sensor(
                fr,
                MAX_PATH_SENSOR_CAP
            )


            slc = self.clean_sensor(
                sl,
                MAX_PATH_SENSOR_CAP
            )


            src = self.clean_sensor(
                sr,
                MAX_PATH_SENSOR_CAP
            )


            self.path_fl_sum += flc

            self.path_fr_sum += frc

            self.path_sl_sum += slc

            self.path_sr_sum += src

            self.path_gz_sum += gz


            self.path_sample_count += 1

            self.path_sample_time += dt


        enough = (

            self.path_sample_time
            >=
            MAX_PATH_SAMPLE_TIME

            and

            self.path_sample_count > 0
        )


        timeout = (

            self.path_total_time

            >=

            MAX_PATH_SAMPLE_TIMEOUT
        )


        if (
            enough
            or
            timeout
        ):

            # ====================================================
            # AVERAGE
            # ====================================================

            if self.path_sample_count > 0:

                n = float(
                    self.path_sample_count
                )


                avg_fl = (
                    self.path_fl_sum
                    /
                    n
                )


                avg_fr = (
                    self.path_fr_sum
                    /
                    n
                )


                avg_sl = (
                    self.path_sl_sum
                    /
                    n
                )


                avg_sr = (
                    self.path_sr_sum
                    /
                    n
                )


                avg_gz = (
                    self.path_gz_sum
                    /
                    n
                )


            else:

                avg_fl = self.path_last_fl
                avg_fr = self.path_last_fr

                avg_sl = self.path_last_sl
                avg_sr = self.path_last_sr

                avg_gz = self.path_last_gz


            # Use standing gyro average as heading bias.
            self.drive_gyro_bias = avg_gz


            (
                direction,
                reason
            ) = self.choose_max_path(
                avg_fl,
                avg_fr,
                avg_sl,
                avg_sr,
                avg_gz
            )


            self.reset_path_samples()


            return self.start_turn(
                direction,
                reason,
                sl,
                sr,
                fl,
                fr
            )


        return (
            0.0,
            0.0
        )


    # ============================================================
    # START FIXED 80 DEG TURN
    # ============================================================

    def start_turn(
        self,
        direction,
        reason,
        sl,
        sr,
        fl,
        fr
    ):

        self.turn_dir = direction


        self.mode = "BRAKE_TURN"


        self.brake_time = 0.0


        self.gyro_sum = 0.0

        self.gyro_samples = 0


        self.turn_angle = 0.0

        self.turn_time = 0.0


        self.corner_timer = 0.0

        self.side_detector_armed = False


        self.reset_drive_controller()


        side = (
            "LEFT"
            if direction > 0
            else
            "RIGHT"
        )


        print()
        print("==================================================")
        print(" FIXED 80 DEG TURN")
        print("==================================================")


        print(
            f"TURN = {side}"
        )


        print(
            f"Reason = {reason}"
        )


        print(
            f"FL={fl:.3f} "
            f"FR={fr:.3f} "
            f"SL={sl:.3f} "
            f"SR={sr:.3f}"
        )


        print(
            "TARGET = 80.0 DEG"
        )


        print("==================================================")
        print()


        return (
            0.0,
            0.0
        )


    # ============================================================
    # BRAKE / ESTIMATE GYRO BIAS
    # ============================================================

    def brake_turn(
        self,
        gz,
        dt
    ):

        self.brake_time += dt


        self.gyro_sum += gz

        self.gyro_samples += 1


        if self.brake_time >= BRAKE_TIME:

            self.gyro_bias = (

                self.gyro_sum

                /

                max(
                    1,
                    self.gyro_samples
                )
            )


            self.drive_gyro_bias = (
                self.gyro_bias
            )


            self.turn_angle = 0.0

            self.turn_time = 0.0


            self.mode = "TURN"


            print(
                "80 DEG TURN START",
                flush=True
            )


        return (
            0.0,
            0.0
        )


    # ============================================================
    # EXACT 80 DEG TURN
    #
    # VERY IMPORTANT:
    #
    # There is NO minimum 1.0 command near the target anymore.
    #
    # That minimum speed was a major reason for 90+ overshoot.
    #
    # This controller:
    #
    # 1. turns quickly far away
    # 2. slows strongly near 80
    # 3. allows command to become ZERO
    # 4. reverses gently if it overshoots
    # ============================================================

    def fixed_turn(
        self,
        gz,
        dt
    ):

        corrected_gz = (
            gz
            -
            self.gyro_bias
        )


        rate_deg = (

            corrected_gz

            *

            180.0

            /

            math.pi
        )


        self.turn_angle += (
            rate_deg
            *
            dt
        )


        self.turn_time += dt


        turned = abs(
            self.turn_angle
        )


        error = (
            TURN_TARGET
            -
            turned
        )


        rate_abs = abs(
            rate_deg
        )


        # ========================================================
        # FINISHED ONLY WHEN:
        #
        # angle correct
        # AND
        # rotation mostly stopped
        #
        # This prevents stopping command at 80 while inertia
        # carries robot to 90.
        # ========================================================

        if (
            abs(error)
            <=
            TURN_TOLERANCE

            and

            rate_abs <= 8.0
        ):

            print()
            print("==================================================")
            print(" 80 DEG TURN LOCKED")
            print("==================================================")


            print(
                f"Angle = "
                f"{turned:.2f}"
            )


            print(
                f"Rate = "
                f"{rate_abs:.2f} deg/s"
            )


            print("==================================================")
            print()


            self.mode = "TURN_STOP"

            self.turn_stop_time = 0.0


            return (
                0.0,
                0.0
            )


        # ========================================================
        # TIMEOUT
        # ========================================================

        if self.turn_time >= TURN_TIMEOUT:

            print(
                f"TURN TIMEOUT "
                f"angle={turned:.2f}",
                flush=True
            )


            self.mode = "TURN_STOP"

            self.turn_stop_time = 0.0


            return (
                0.0,
                0.0
            )


        # ========================================================
        # OVERSHOOT / RESCUE
        #
        # If > 80, rotate backwards.
        # ========================================================

        if error < 0.0:

            desired_dir = (
                -self.turn_dir
            )


            command = (

                0.15

                +

                0.08
                *
                abs(error)
            )


            command = self.clamp(
                command,
                0.15,
                TURN_REVERSE_MAX
            )


            # More aggressive rescue before anything close to 90.
            if turned >= TURN_RESCUE_ANGLE:

                command = max(
                    command,
                    0.65
                )


        # ========================================================
        # STILL NEED MORE ANGLE
        # ========================================================

        else:

            desired_dir = (
                self.turn_dir
            )


            raw_command = (

                TURN_KP
                *
                error

                -

                TURN_KD
                *
                rate_abs
            )


            # ----------------------------------------------------
            # FAR FROM TARGET
            # ----------------------------------------------------

            if error > 25.0:

                command = self.clamp(
                    raw_command,
                    1.10,
                    TURN_MAX_SPEED
                )


            # ----------------------------------------------------
            # MID RANGE
            # ----------------------------------------------------

            elif error > 12.0:

                command = self.clamp(
                    raw_command,
                    0.65,
                    1.55
                )


            # ----------------------------------------------------
            # APPROACH
            # ----------------------------------------------------

            elif error > 5.0:

                command = self.clamp(
                    raw_command,
                    0.30,
                    0.90
                )


            # ----------------------------------------------------
            # FINAL 5 DEGREES
            #
            # Allow speed to become almost zero.
            # ----------------------------------------------------

            elif error > TURN_TOLERANCE:

                command = self.clamp(
                    raw_command,
                    0.0,
                    0.45
                )


                # If nearly stopped but still short,
                # give a tiny controlled nudge.
                if (
                    command < 0.12

                    and

                    rate_abs < 4.0

                    and

                    error > 1.0
                ):

                    command = 0.15


            # ----------------------------------------------------
            # Angle is in tolerance but robot still rotating.
            #
            # Command ZERO and let rate die.
            # ----------------------------------------------------

            else:

                command = 0.0


        # ========================================================
        # EXTREME PROTECTION
        #
        # At 84 degrees do NOT continue original direction.
        # Force reverse correction.
        # ========================================================

        if turned >= TURN_HARD_LIMIT:

            desired_dir = (
                -self.turn_dir
            )


            command = 0.75


        # ========================================================
        # MOTOR DIRECTION
        # ========================================================

        if desired_dir > 0:

            # LEFT

            left = -command

            right = +command


            physical = "LEFT"


        else:

            # RIGHT

            left = +command

            right = -command


            physical = "RIGHT"


        # ========================================================
        # LOG
        # ========================================================

        now = time.monotonic()


        if (
            now
            -
            self.last_log
            >
            0.06
        ):

            self.last_log = now


            wanted = (
                "LEFT"
                if self.turn_dir > 0
                else
                "RIGHT"
            )


            print(
                f"TURN wanted={wanted} "
                f"motor={physical} "
                f"angle={turned:6.2f} "
                f"error={error:+6.2f} "
                f"rate={rate_abs:6.1f} "
                f"cmd={command:.2f}",
                flush=True
            )


        return (
            left,
            right
        )


    # ============================================================
    # TURN STOP
    # ============================================================

    def turn_stop(
        self,
        dt
    ):

        self.turn_stop_time += dt


        if (
            self.turn_stop_time
            >=
            TURN_STOP_TIME
        ):

            self.mode = "SETTLE"


            self.settle_time = 0.0


            self.corner_timer = 0.0


            self.reset_drive_controller()


            print(
                "TURN STOP -> STABILIZE",
                flush=True
            )


        return (
            0.0,
            0.0
        )


    # ============================================================
    # MAIN STEP
    # ============================================================

    def step(
        self,
        data
    ):

        # ========================================================
        # RAW SENSORS
        # ========================================================

        raw_sl = float(
            data["sl"]
        )


        raw_sr = float(
            data["sr"]
        )


        raw_fl = float(
            data["fl"]
        )


        raw_fr = float(
            data["fr"]
        )


        gz = float(
            data["gyro"][2]
        )


        try:

            dt = float(
                data.get(
                    "dt",
                    0.002
                )
            )


        except (
            TypeError,
            ValueError
        ):

            dt = 0.002


        if not (
            0.0005
            <
            dt
            <
            0.02
        ):

            dt = 0.002


        self.sim_time += dt


        # ========================================================
        # FILTER
        # ========================================================

        self.update_filters(
            raw_sl,
            raw_sr,
            raw_fl,
            raw_fr
        )


        sl = self.f_sl

        sr = self.f_sr

        fl = self.f_fl

        fr = self.f_fr


        # ========================================================
        # STATE MACHINE
        # ========================================================

        if self.mode == "PATH_SAMPLE":

            return self.path_sample(
                raw_sl,
                raw_sr,
                raw_fl,
                raw_fr,
                gz,
                dt
            )


        if self.mode == "BRAKE_TURN":

            return self.brake_turn(
                gz,
                dt
            )


        if self.mode == "TURN":

            return self.fixed_turn(
                gz,
                dt
            )


        if self.mode == "TURN_STOP":

            return self.turn_stop(
                dt
            )


        # ========================================================
        # SETTLE
        #
        # Strong centering controller is ALREADY active here.
        #
        # Therefore stabilization begins immediately after turn.
        # ========================================================

        if self.mode == "SETTLE":

            self.settle_time += dt


            (
                left,
                right,
                drive_mode,
                error,
                front_mode,
                side_mode
            ) = self.straight_drive(
                sl,
                sr,
                fl,
                fr,
                gz,
                dt,
                SETTLE_SPEED,
                True
            )


            if (
                self.settle_time
                >=
                SETTLE_TIME
            ):

                self.mode = "DRIVE"


                self.completed_turns += 1


                self.turn_lockout_until = (

                    self.sim_time

                    +

                    POST_TURN_LOCKOUT
                )


                self.side_detector_armed = False

                self.corner_timer = 0.0


                # New corridor heading starts here.
                self.reset_drive_controller()


                print()
                print("==================================================")
                print(" 80 DEG TURN COMPLETE -> DRIVE")
                print("==================================================")


                print(
                    f"Completed turns = "
                    f"{self.completed_turns}"
                )


                print(
                    f"Lockout = "
                    f"{POST_TURN_LOCKOUT:.2f}s"
                )


                print("==================================================")
                print()


            return (
                left,
                right
            )


        # ========================================================
        # MAZE ENTRY
        # ========================================================

        if not self.inside_maze:

            if (
                raw_sl <= INSIDE_SIDE_DIST

                or

                raw_sr <= INSIDE_SIDE_DIST
            ):

                self.inside_maze = True

                self.first_front_count = 0


                print(
                    "BOT INSIDE MAZE",
                    flush=True
                )


        # ========================================================
        # LOCKOUT
        # ========================================================

        lockout_active = (

            self.sim_time

            <

            self.turn_lockout_until
        )


        if lockout_active:

            lock_remaining = (

                self.turn_lockout_until

                -

                self.sim_time
            )


        else:

            lock_remaining = 0.0


        corner_now = False


        # ========================================================
        # FIRST TURN
        #
        # NO 90 DEG MAX SCAN.
        #
        # When original first trigger occurs:
        #
        # stop
        # sample sensors
        # choose MAX
        # turn exactly 80
        # ========================================================

        if self.completed_turns == 0:

            navigation_mode = "FIRST_MAX_80"


            if self.inside_maze:

                first_close = (

                    raw_fl <= FIRST_FRONT_DIST

                    or

                    raw_fr <= FIRST_FRONT_DIST
                )


                if first_close:

                    self.first_front_count += 1


                else:

                    self.first_front_count = 0


                if (
                    self.first_front_count

                    >=

                    FIRST_FRONT_DEBOUNCE
                ):

                    self.first_front_count = 0


                    return self.start_path_sample(
                        raw_sl,
                        raw_sr,
                        raw_fl,
                        raw_fr,
                        gz,
                        True
                    )


            speed = self.drive_speed(
                sl,
                sr,
                False
            )


            speed = self.predictive_speed(
                sl,
                sr,
                dt,
                speed
            )


            # Keep first entry collision logic gentle.
            (
                left,
                right,
                drive_mode,
                error,
                front_mode,
                side_mode
            ) = self.straight_drive(
                sl,
                sr,
                fl,
                fr,
                gz,
                dt,
                speed,
                False
            )


        # ========================================================
        # AFTER FIRST TURN
        # ========================================================

        else:

            navigation_mode = "CENTER_MAX80"


            # ====================================================
            # ARM CORNER DETECTOR
            # ====================================================

            if not self.side_detector_armed:

                if (
                    sl > SIDE_ARM_DIST

                    and

                    sr > SIDE_ARM_DIST
                ):

                    self.side_detector_armed = True

                    self.corner_timer = 0.0


                    print(
                        "CORNER ARMED "
                        f"SL={sl:.3f} "
                        f"SR={sr:.3f}",
                        flush=True
                    )


            # ====================================================
            # CORNER CONFIRMATION
            # ====================================================

            corner_now = self.corner_condition(
                sl,
                sr
            )


            if (
                self.side_detector_armed

                and

                not lockout_active
            ):

                if corner_now:

                    self.corner_timer += dt


                else:

                    self.corner_timer = 0.0


                if (
                    self.corner_timer

                    >=

                    CORNER_CONFIRM_TIME
                ):

                    return self.start_path_sample(
                        raw_sl,
                        raw_sr,
                        raw_fl,
                        raw_fr,
                        gz,
                        False
                    )


            else:

                self.corner_timer = 0.0


            # ====================================================
            # FRONT DEAD END
            #
            # BOTH only.
            #
            # Single grazing sensor cannot create a turn.
            # ====================================================

            front_dead_end = (

                raw_fl <= FRONT_STOP_DIST

                and

                raw_fr <= FRONT_STOP_DIST
            )


            if (
                front_dead_end

                and

                not lockout_active
            ):

                return self.start_path_sample(
                    raw_sl,
                    raw_sr,
                    raw_fl,
                    raw_fr,
                    gz,
                    False
                )


            # ====================================================
            # ABSOLUTE SIDE EMERGENCY
            # ====================================================

            side_emergency = (

                raw_sl <= SIDE_EMERGENCY_DIST

                or

                raw_sr <= SIDE_EMERGENCY_DIST
            )


            if side_emergency:

                if lockout_active:

                    # Immediate stop.
                    return (
                        0.0,
                        0.0
                    )


                return self.start_path_sample(
                    raw_sl,
                    raw_sr,
                    raw_fl,
                    raw_fr,
                    gz,
                    False
                )


            # ====================================================
            # SPEED DEPENDS ON CENTERING QUALITY
            # ====================================================

            speed = self.drive_speed(
                sl,
                sr,
                lockout_active
            )


            # ====================================================
            # PREDICTIVE SPEED REDUCTION
            # ====================================================

            speed = self.predictive_speed(
                sl,
                sr,
                dt,
                speed
            )


            # ====================================================
            # STRAIGHT CENTER CONTROL
            # ====================================================

            (
                left,
                right,
                drive_mode,
                error,
                front_mode,
                side_mode
            ) = self.straight_drive(
                sl,
                sr,
                fl,
                fr,
                gz,
                dt,
                speed,
                True
            )


        # ========================================================
        # LOG
        # ========================================================

        now = time.monotonic()


        if (
            now
            -
            self.last_log
            >
            0.09
        ):

            self.last_log = now


            center_diff = (
                sr
                -
                sl
            )


            print(
                f"DRIVE "
                f"{drive_mode:7s} "
                f"mode={navigation_mode} "
                f"FL={fl:.3f} "
                f"FR={fr:.3f} "
                f"SL={sl:.3f} "
                f"SR={sr:.3f} "
                f"DIFF={center_diff:+.3f} "
                f"yaw={self.drive_yaw_deg:+.2f} "
                f"speed={speed:.2f} "
                f"front={front_mode} "
                f"side={side_mode} "
                f"corner={corner_now} "
                f"LOCK={lock_remaining:.2f}s "
                f"L={left:+.2f} "
                f"R={right:+.2f}",
                flush=True
            )


        return (
            left,
            right
        )


# ================================================================
# CONTROLLER
# ================================================================

controller = Controller()


# ================================================================
# MQTT CALLBACK
# ================================================================

def on_message(
    client,
    userdata,
    msg
):

    try:

        data = json.loads(
            msg.payload.decode()
        )


        left, right = controller.step(
            data
        )


        client.publish(
            TOPIC_WHEEL_VEL,
            json.dumps(
                {
                    "left": float(left),
                    "right": float(right)
                }
            )
        )


    except Exception as error:

        print(
            "ERROR:",
            error,
            flush=True
        )


        client.publish(
            TOPIC_WHEEL_VEL,
            json.dumps(
                {
                    "left": 0.0,
                    "right": 0.0
                }
            )
        )


# ================================================================
# MAIN
# ================================================================

def main():

    try:

        client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2
        )


    except AttributeError:

        client = mqtt.Client()


    client.on_message = on_message


    client.connect(
        MQTT_HOST,
        MQTT_PORT
    )


    client.subscribe(
        TOPIC_SENSORS
    )


    print()
    print("==================================================")
    print(" PACBOT CENTERED + EXACT 80 DEG")
    print("==================================================")
    print()

    print("STRAIGHT CONTROL")
    print("----------------")
    print("Both walls:")
    print("  target = SL == SR")
    print("  CENTER KP =", CENTER_KP)
    print("  heading lock enabled")
    print()

    print("One wall:")
    print(
        f"  target distance = "
        f"{ONE_WALL_TARGET}"
    )
    print()

    print("SPEED")
    print("-----")
    print(
        f"centered clear = "
        f"{BASE_SPEED}"
    )
    print(
        f"one wall = "
        f"{ONE_WALL_SPEED}"
    )
    print(
        f"settle = "
        f"{SETTLE_SPEED}"
    )
    print()

    print("COLLISION PROTECTION")
    print("--------------------")
    print(
        f"front warning = "
        f"{FRONT_WARNING_DIST}"
    )
    print(
        f"front danger = "
        f"{FRONT_DANGER_DIST}"
    )
    print(
        f"front stop = "
        f"{FRONT_STOP_DIST}"
    )
    print(
        f"side warning = "
        f"{SIDE_WARNING_DIST}"
    )
    print(
        f"side danger = "
        f"{SIDE_DANGER_DIST}"
    )
    print(
        f"side emergency = "
        f"{SIDE_EMERGENCY_DIST}"
    )
    print()

    print("TURN")
    print("----")
    print(
        f"target = "
        f"{TURN_TARGET} DEG"
    )
    print(
        f"tolerance = "
        f"{TURN_TOLERANCE} DEG"
    )
    print(
        f"overshoot rescue = "
        f"{TURN_RESCUE_ANGLE} DEG"
    )
    print(
        f"absolute protection = "
        f"{TURN_HARD_LIMIT} DEG"
    )
    print()

    print("IMPORTANT")
    print("---------")
    print("NO physical 90-degree max scan")
    print("MAX path is measured while stopped")
    print("ALL turns use the same 80-degree controller")
    print()

    print("==================================================")
    print()


    client.loop_forever()


# ================================================================
# RUN
# ================================================================

if __name__ == "__main__":

    try:

        main()


    except KeyboardInterrupt:

        print(
            "\nStopped.",
            flush=True
        )
