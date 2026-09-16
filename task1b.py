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
# SPEED
# ================================================================

BASE_SPEED = 11.25

FAST_SPEED = 9.0

MEDIUM_SPEED = 7.0

RECOVERY_SPEED = 4.5

HARD_RECOVERY_SPEED = 3.2

ONE_WALL_SPEED = 7.0

OPEN_SPEED = 9.0

SETTLE_SPEED = 4.0

LOCKOUT_SPEED = 4.5

MAX_WHEEL = 13.5


# ================================================================
# ACCELERATION
# ================================================================

ACCEL_RATE = 20.0


# ================================================================
# SENSOR FILTER
# ================================================================

SENSOR_ALPHA = 0.20


# ================================================================
# MAZE ENTRY / FIRST TURN
# ================================================================

INSIDE_SIDE_DIST = 0.12

FIRST_FRONT_DIST = 0.14

FIRST_FRONT_DEBOUNCE = 4


# ================================================================
# TURN
#
# UNCHANGED
# ================================================================

TURN_TARGET = 80.0

TURN_TOLERANCE = 0.55

TURN_KP = 0.052

TURN_KD = 0.012

TURN_MAX_SPEED = 2.60

TURN_REVERSE_MAX = 0.70

TURN_RESCUE_ANGLE = 82.0

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
# ================================================================

SIDE_TURN_DIST = 0.12

SIDE_CONFIRM_DIST = 0.16

SIDE_ARM_DIST = 0.18

CORNER_CONFIRM_TIME = 0.025


# ================================================================
# FRONT CONTROL
#
# FL < .15 -> RIGHT
# FR < .15 -> LEFT
#
# BOTH > .15 -> normal PID
#
# BOTH LOW:
# lower sensor decides.
#
# nearly equal -> straight
# ================================================================

FRONT_TARGET = 0.15

FRONT_RELEASE = 0.155

FRONT_DIFFERENCE_MARGIN = 0.018

FRONT_AVOID_MIN = 0.70

FRONT_AVOID_GAIN = 12.0

FRONT_AVOID_MAX = 2.20

FRONT_AVOID_SPEED = 6.5

FRONT_DIRECTION_HOLD = 0.08


# ================================================================
# MAX PATH
# ================================================================

MAX_PATH_SAMPLE_TIME = 0.045

MAX_PATH_SAMPLE_TIMEOUT = 0.14

MAX_PATH_SENSOR_CAP = 1.0

MAX_PATH_GYRO_LIMIT = 0.30

MAX_PATH_FRONT_WEIGHT = 0.65

MAX_PATH_SIDE_WEIGHT = 0.35

MAX_PATH_MIN_DIFFERENCE = 0.035


# ================================================================
# WALL DETECTION
# ================================================================

WALL_MIN = 0.025

WALL_MAX = 0.45


# ================================================================
# ONE WALL FOLLOWING
# ================================================================

ONE_WALL_TARGET = 0.23


# ================================================================
# CENTER PID
# ================================================================

CENTER_KP = 6.5

CENTER_KD = 0.18

CENTER_KI = 0.0

ONE_WALL_KP = 4.5

CENTER_ERROR_LIMIT = 0.18

CENTER_DEADBAND = 0.002

ERROR_FILTER_ALPHA = 0.12

DERIVATIVE_LIMIT = 0.35


# ================================================================
# GYRO
# ================================================================

GYRO_RATE_GAIN = 0.006

OPEN_HEADING_KP = 0.030


# ================================================================
# STEERING
# ================================================================

MAX_CORRECTION = 3.0

NORMAL_CORRECTION_SLEW = 14.0

FRONT_CORRECTION_SLEW = 35.0


# ================================================================
# SIDE WALL SAFETY
# ================================================================

SIDE_WARNING_DIST = 0.17

SIDE_DANGER_DIST = 0.14


# ================================================================
# ONLY CHANGE
#
# OLD = 0.11
#
# NEW = 0.14
#
# +0.03 WALL STOP DISTANCE
# ================================================================

SIDE_HARD_DIST = 0.14


SIDE_BARRIER_GAIN = 10.0

SIDE_BARRIER_MAX = 1.30


# ================================================================
# SIDE SPEED CONTROL
# ================================================================

SIDE_WARNING_SPEED = 5.5

SIDE_DANGER_SPEED = 3.5

SIDE_HARD_SPEED = 2.0


# ================================================================
# PREDICTIVE SIDE
# ================================================================

PREDICT_HORIZON = 0.050

MAX_CLOSING_RATE = 0.80


# ================================================================
# CENTER SPEED
# ================================================================

CENTER_FAST_DIFF = 0.010

CENTER_GOOD_DIFF = 0.025

CENTER_MED_DIFF = 0.050

CENTER_BAD_DIFF = 0.090

CENTER_STABLE_TIME_FOR_MAX = 0.12


# ================================================================
# CONTROLLER
# ================================================================

class Controller:

    def __init__(self):

        self.mode = "DRIVE"

        self.inside_maze = False

        self.completed_turns = 0

        self.first_front_count = 0


        # +1 LEFT
        # -1 RIGHT
        self.turn_dir = +1


        # ========================================================
        # FILTERS
        # ========================================================

        self.filters_ready = False

        self.f_sl = 0.0
        self.f_sr = 0.0

        self.f_fl = 0.0
        self.f_fr = 0.0


        # ========================================================
        # PID
        # ========================================================

        self.filtered_error = 0.0

        self.previous_filtered_error = 0.0

        self.have_error = False

        self.previous_correction = 0.0


        # ========================================================
        # HEADING
        # ========================================================

        self.drive_yaw_deg = 0.0

        self.drive_gyro_bias = 0.0


        # ========================================================
        # SPEED
        # ========================================================

        self.command_speed = 0.0

        self.center_stable_time = 0.0


        # ========================================================
        # FRONT AVOIDANCE
        # ========================================================

        self.front_avoid_dir = 0

        self.front_hold_time = 0.0


        # ========================================================
        # SIDE PREDICTION
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


        # ========================================================
        # TURN
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

        self.last_log = 0.0


    # ============================================================
    # HELPERS
    # ============================================================

    @staticmethod
    def clamp(value, low, high):

        return max(
            low,
            min(
                high,
                value
            )
        )


    @staticmethod
    def clean_sensor(value, cap):

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
    # SENSOR FILTER
    # ============================================================

    def update_filters(
        self,
        sl,
        sr,
        fl,
        fr
    ):

        sl = self.clean_sensor(sl, 1.0)

        sr = self.clean_sensor(sr, 1.0)

        fl = self.clean_sensor(fl, 1.0)

        fr = self.clean_sensor(fr, 1.0)


        if not self.filters_ready:

            self.f_sl = sl
            self.f_sr = sr

            self.f_fl = fl
            self.f_fr = fr

            self.filters_ready = True

            return


        a = SENSOR_ALPHA


        self.f_sl = (
            a * sl
            +
            (1.0 - a) * self.f_sl
        )


        self.f_sr = (
            a * sr
            +
            (1.0 - a) * self.f_sr
        )


        self.f_fl = (
            a * fl
            +
            (1.0 - a) * self.f_fl
        )


        self.f_fr = (
            a * fr
            +
            (1.0 - a) * self.f_fr
        )


    # ============================================================
    # RESET DRIVE
    # ============================================================

    def reset_drive_controller(self):

        self.filtered_error = 0.0

        self.previous_filtered_error = 0.0

        self.have_error = False

        self.previous_correction = 0.0


        self.drive_yaw_deg = 0.0


        self.prev_sl = None

        self.prev_sr = None


        self.center_stable_time = 0.0


        self.front_avoid_dir = 0

        self.front_hold_time = 0.0


    # ============================================================
    # RESET PATH
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
    # SPEED RAMP
    # ============================================================

    def ramp_speed(
        self,
        requested_speed,
        dt
    ):

        if requested_speed < self.command_speed:

            self.command_speed = requested_speed


        else:

            self.command_speed += (
                ACCEL_RATE
                *
                dt
            )


            if self.command_speed > requested_speed:

                self.command_speed = requested_speed


        return self.command_speed


    # ============================================================
    # SIDE PREDICTION
    # ============================================================

    def predict_sides(
        self,
        sl,
        sr,
        dt
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

            return (
                sl,
                sr
            )


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


        return (
            predicted_sl,
            predicted_sr
        )


    # ============================================================
    # FRONT AVOIDANCE
    #
    # FL < .15 -> RIGHT
    #
    # FR < .15 -> LEFT
    # ============================================================

    def front_avoidance(
        self,
        fl,
        fr,
        dt
    ):

        self.front_hold_time = max(
            0.0,
            self.front_hold_time - dt
        )


        fl_low = (
            fl < FRONT_TARGET
        )


        fr_low = (
            fr < FRONT_TARGET
        )


        # ========================================================
        # BOTH SAFE
        # ========================================================

        if (
            fl >= FRONT_RELEASE
            and
            fr >= FRONT_RELEASE
        ):

            self.front_avoid_dir = 0

            self.front_hold_time = 0.0


            return (
                0.0,
                False,
                "FRONT_CLEAR"
            )


        desired_dir = 0


        # ========================================================
        # FL LOW -> RIGHT
        # ========================================================

        if (
            fl_low
            and
            not fr_low
        ):

            desired_dir = +1


        # ========================================================
        # FR LOW -> LEFT
        # ========================================================

        elif (
            fr_low
            and
            not fl_low
        ):

            desired_dir = -1


        # ========================================================
        # BOTH LOW
        # ========================================================

        elif (
            fl_low
            and
            fr_low
        ):

            if (
                fl
                <
                fr - FRONT_DIFFERENCE_MARGIN
            ):

                desired_dir = +1


            elif (
                fr
                <
                fl - FRONT_DIFFERENCE_MARGIN
            ):

                desired_dir = -1


            else:

                desired_dir = 0


        else:

            desired_dir = self.front_avoid_dir


        # ========================================================
        # EQUAL
        # ========================================================

        if desired_dir == 0:

            self.front_avoid_dir = 0


            return (
                0.0,
                False,
                "FRONT_EQUAL"
            )


        # ========================================================
        # FIRST ACTIVATION
        # ========================================================

        if self.front_avoid_dir == 0:

            self.front_avoid_dir = desired_dir

            self.front_hold_time = (
                FRONT_DIRECTION_HOLD
            )


        # ========================================================
        # SWITCH DIRECTION AFTER HOLD
        # ========================================================

        elif (
            desired_dir
            !=
            self.front_avoid_dir

            and

            self.front_hold_time <= 0.0
        ):

            self.front_avoid_dir = desired_dir

            self.front_hold_time = (
                FRONT_DIRECTION_HOLD
            )


        # ========================================================
        # FL LOW
        # ========================================================

        if self.front_avoid_dir > 0:

            error = max(
                0.0,
                FRONT_TARGET
                -
                fl
            )


            correction = (
                FRONT_AVOID_MIN

                +

                FRONT_AVOID_GAIN
                *
                error
            )


            correction = self.clamp(
                correction,
                FRONT_AVOID_MIN,
                FRONT_AVOID_MAX
            )


            return (
                +correction,
                True,
                "FL_LOW_RIGHT"
            )


        # ========================================================
        # FR LOW
        # ========================================================

        error = max(
            0.0,
            FRONT_TARGET
            -
            fr
        )


        correction = (
            FRONT_AVOID_MIN

            +

            FRONT_AVOID_GAIN
            *
            error
        )


        correction = self.clamp(
            correction,
            FRONT_AVOID_MIN,
            FRONT_AVOID_MAX
        )


        return (
            -correction,
            True,
            "FR_LOW_LEFT"
        )


    # ============================================================
    # SIDE BARRIER
    # ============================================================

    def side_barrier(
        self,
        sl,
        sr
    ):

        correction = 0.0


        # ========================================================
        # LEFT CLOSE -> RIGHT
        # ========================================================

        if sl < SIDE_WARNING_DIST:

            amount = (
                SIDE_WARNING_DIST
                -
                sl
            )


            correction += (
                SIDE_BARRIER_GAIN
                *
                amount
            )


        # ========================================================
        # RIGHT CLOSE -> LEFT
        # ========================================================

        if sr < SIDE_WARNING_DIST:

            amount = (
                SIDE_WARNING_DIST
                -
                sr
            )


            correction -= (
                SIDE_BARRIER_GAIN
                *
                amount
            )


        return self.clamp(
            correction,
            -SIDE_BARRIER_MAX,
            SIDE_BARRIER_MAX
        )


    # ============================================================
    # CORNER
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
            nearest <= SIDE_TURN_DIST

            and

            farther <= SIDE_CONFIRM_DIST
        )


    # ============================================================
    # DRIVE SPEED
    # ============================================================

    def choose_drive_speed(
        self,
        sl,
        sr,
        predicted_sl,
        predicted_sr,
        dt,
        lockout_active
    ):

        left_wall = self.wall_visible(
            sl
        )


        right_wall = self.wall_visible(
            sr
        )


        nearest = min(
            sl,
            sr
        )


        predicted_nearest = min(
            predicted_sl,
            predicted_sr
        )


        # ========================================================
        # BOTH WALLS
        # ========================================================

        if (
            left_wall
            and
            right_wall
        ):

            diff = abs(
                sr
                -
                sl
            )


            if (
                diff <= CENTER_FAST_DIFF

                and

                nearest > SIDE_WARNING_DIST
            ):

                self.center_stable_time += dt


            else:

                self.center_stable_time = 0.0


            if (
                diff <= CENTER_FAST_DIFF

                and

                self.center_stable_time
                >=
                CENTER_STABLE_TIME_FOR_MAX
            ):

                speed = BASE_SPEED


            elif diff <= CENTER_GOOD_DIFF:

                speed = FAST_SPEED


            elif diff <= CENTER_MED_DIFF:

                speed = MEDIUM_SPEED


            elif diff <= CENTER_BAD_DIFF:

                speed = RECOVERY_SPEED


            else:

                speed = HARD_RECOVERY_SPEED


        # ========================================================
        # ONE WALL
        # ========================================================

        elif (
            left_wall
            or
            right_wall
        ):

            self.center_stable_time = 0.0

            speed = ONE_WALL_SPEED


        # ========================================================
        # OPEN
        # ========================================================

        else:

            self.center_stable_time = 0.0

            speed = OPEN_SPEED


        # ========================================================
        # SIDE WALL DISTANCE
        #
        # HARD DISTANCE NOW .14
        # ========================================================

        if nearest <= SIDE_HARD_DIST:

            speed = min(
                speed,
                SIDE_HARD_SPEED
            )


        elif nearest <= SIDE_DANGER_DIST:

            speed = min(
                speed,
                SIDE_DANGER_SPEED
            )


        elif nearest <= SIDE_WARNING_DIST:

            speed = min(
                speed,
                SIDE_WARNING_SPEED
            )


        # ========================================================
        # PREDICTED WALL DISTANCE
        # ========================================================

        if predicted_nearest <= SIDE_HARD_DIST:

            speed = min(
                speed,
                SIDE_HARD_SPEED
            )


        elif predicted_nearest <= SIDE_DANGER_DIST:

            speed = min(
                speed,
                SIDE_DANGER_SPEED
            )


        elif predicted_nearest <= SIDE_WARNING_DIST:

            speed = min(
                speed,
                SIDE_WARNING_SPEED
            )


        if lockout_active:

            speed = min(
                speed,
                LOCKOUT_SPEED
            )


        return speed


    # ============================================================
    # STRAIGHT DRIVE
    # ============================================================

    def straight_drive(
        self,
        sl,
        sr,
        fl,
        fr,
        gz,
        dt,
        requested_speed
    ):

        left_wall = self.wall_visible(
            sl
        )


        right_wall = self.wall_visible(
            sr
        )


        both_walls = (
            left_wall
            and
            right_wall
        )


        # ========================================================
        # GYRO
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


        # ========================================================
        # HEADING
        # ========================================================

        if both_walls:

            self.drive_yaw_deg *= 0.92


        else:

            self.drive_yaw_deg += (
                gyro_rate_deg
                *
                dt
            )


            self.drive_yaw_deg = self.clamp(
                self.drive_yaw_deg,
                -6.0,
                6.0
            )


        # ========================================================
        # WALL ERROR
        # ========================================================

        if both_walls:

            raw_error = (
                sr
                -
                sl
            )


            kp = CENTER_KP

            drive_mode = "CENTER"


        elif left_wall:

            raw_error = (
                ONE_WALL_TARGET
                -
                sl
            )


            kp = ONE_WALL_KP

            drive_mode = "LEFT"


        elif right_wall:

            raw_error = (
                sr
                -
                ONE_WALL_TARGET
            )


            kp = ONE_WALL_KP

            drive_mode = "RIGHT"


        else:

            raw_error = 0.0

            kp = 0.0

            drive_mode = "OPEN"


        raw_error = self.clamp(
            raw_error,
            -CENTER_ERROR_LIMIT,
            CENTER_ERROR_LIMIT
        )


        # ========================================================
        # FILTER ERROR
        # ========================================================

        if not self.have_error:

            self.filtered_error = raw_error

            self.previous_filtered_error = raw_error

            self.have_error = True


        else:

            self.filtered_error = (
                ERROR_FILTER_ALPHA
                *
                raw_error

                +

                (
                    1.0
                    -
                    ERROR_FILTER_ALPHA
                )
                *
                self.filtered_error
            )


        if abs(self.filtered_error) < CENTER_DEADBAND:

            self.filtered_error = 0.0


        # ========================================================
        # DERIVATIVE
        # ========================================================

        if dt > 0.0001:

            derivative = (
                self.filtered_error
                -
                self.previous_filtered_error
            ) / dt


        else:

            derivative = 0.0


        derivative = self.clamp(
            derivative,
            -DERIVATIVE_LIMIT,
            DERIVATIVE_LIMIT
        )


        self.previous_filtered_error = (
            self.filtered_error
        )


        # ========================================================
        # WALL CONTROL
        # ========================================================

        wall_correction = (
            kp
            *
            self.filtered_error

            +

            CENTER_KD
            *
            derivative
        )


        # ========================================================
        # GYRO
        # ========================================================

        if both_walls:

            gyro_correction = (
                GYRO_RATE_GAIN
                *
                gyro_rate_deg
            )


        else:

            gyro_correction = (
                OPEN_HEADING_KP
                *
                self.drive_yaw_deg

                +

                GYRO_RATE_GAIN
                *
                gyro_rate_deg
            )


        # ========================================================
        # SIDE WALL
        # ========================================================

        side_correction = self.side_barrier(
            sl,
            sr
        )


        # ========================================================
        # FRONT .15 AVOIDANCE
        # ========================================================

        (
            front_correction,
            front_active,
            front_mode
        ) = self.front_avoidance(
            fl,
            fr,
            dt
        )


        # ========================================================
        # FRONT PRIORITY
        # ========================================================

        if front_active:

            normal_correction = (
                0.35
                *
                wall_correction

                +

                0.30
                *
                gyro_correction

                +

                0.50
                *
                side_correction
            )


            correction = (
                normal_correction
                +
                front_correction
            )


            requested_speed = min(
                requested_speed,
                FRONT_AVOID_SPEED
            )


        # ========================================================
        # NORMAL PID
        # ========================================================

        else:

            correction = (
                wall_correction
                +
                gyro_correction
                +
                side_correction
            )


        correction = self.clamp(
            correction,
            -MAX_CORRECTION,
            MAX_CORRECTION
        )


        # ========================================================
        # SLEW
        # ========================================================

        if front_active:

            slew = FRONT_CORRECTION_SLEW


        else:

            slew = NORMAL_CORRECTION_SLEW


        max_change = (
            slew
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
        # SPEED
        # ========================================================

        speed = self.ramp_speed(
            requested_speed,
            dt
        )


        # ========================================================
        # WHEELS
        # ========================================================

        left = (
            speed
            +
            final_correction
        )


        right = (
            speed
            -
            final_correction
        )


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
            self.filtered_error,
            front_mode,
            final_correction,
            speed
        )


    # ============================================================
    # PATH SCORE
    # ============================================================

    def calculate_path_scores(
        self,
        fl,
        fr,
        sl,
        sr
    ):

        fl = self.clean_sensor(
            fl,
            MAX_PATH_SENSOR_CAP
        )


        fr = self.clean_sensor(
            fr,
            MAX_PATH_SENSOR_CAP
        )


        sl = self.clean_sensor(
            sl,
            MAX_PATH_SENSOR_CAP
        )


        sr = self.clean_sensor(
            sr,
            MAX_PATH_SENSOR_CAP
        )


        left_score = (
            MAX_PATH_FRONT_WEIGHT
            *
            fl

            +

            MAX_PATH_SIDE_WEIGHT
            *
            sl
        )


        right_score = (
            MAX_PATH_FRONT_WEIGHT
            *
            fr

            +

            MAX_PATH_SIDE_WEIGHT
            *
            sr
        )


        return (
            left_score,
            right_score
        )


    # ============================================================
    # MAX PATH
    # ============================================================

    def choose_max_path(
        self,
        fl,
        fr,
        sl,
        sr,
        gz
    ):

        fl = self.clean_sensor(
            fl,
            MAX_PATH_SENSOR_CAP
        )


        fr = self.clean_sensor(
            fr,
            MAX_PATH_SENSOR_CAP
        )


        sl = self.clean_sensor(
            sl,
            MAX_PATH_SENSOR_CAP
        )


        sr = self.clean_sensor(
            sr,
            MAX_PATH_SENSOR_CAP
        )


        (
            left_score,
            right_score
        ) = self.calculate_path_scores(
            fl,
            fr,
            sl,
            sr
        )


        difference = (
            left_score
            -
            right_score
        )


        print()
        print("==================================================")
        print(" MAX PATH")
        print("==================================================")


        print(
            f"FL={fl:.3f} "
            f"FR={fr:.3f} "
            f"SL={sl:.3f} "
            f"SR={sr:.3f}"
        )


        print(
            f"LEFT={left_score:.3f} "
            f"RIGHT={right_score:.3f}"
        )


        # ========================================================
        # LEFT
        # ========================================================

        if difference > MAX_PATH_MIN_DIFFERENCE:

            print("MAX PATH = LEFT")
            print("==================================================")
            print()


            return (
                +1,
                "MAX LEFT"
            )


        # ========================================================
        # RIGHT
        # ========================================================

        if difference < -MAX_PATH_MIN_DIFFERENCE:

            print("MAX PATH = RIGHT")
            print("==================================================")
            print()


            return (
                -1,
                "MAX RIGHT"
            )


        # ========================================================
        # FRONT TIE
        # ========================================================

        if fl > fr + 0.02:

            print("TIE -> LEFT")
            print("==================================================")
            print()


            return (
                +1,
                "FL MORE OPEN"
            )


        if fr > fl + 0.02:

            print("TIE -> RIGHT")
            print("==================================================")
            print()


            return (
                -1,
                "FR MORE OPEN"
            )


        # ========================================================
        # SIDE FALLBACK
        # ========================================================

        if sl < sr:

            print("FALLBACK -> RIGHT")
            print("==================================================")
            print()


            return (
                -1,
                "SL CLOSER -> RIGHT"
            )


        else:

            print("FALLBACK -> LEFT")
            print("==================================================")
            print()


            return (
                +1,
                "SR CLOSER -> LEFT"
            )


    # ============================================================
    # START PATH SAMPLE
    # ============================================================

    def start_path_sample(
        self,
        sl,
        sr,
        fl,
        fr,
        gz
    ):

        self.mode = "PATH_SAMPLE"

        self.command_speed = 0.0

        self.corner_timer = 0.0


        self.reset_path_samples()


        self.path_last_fl = fl
        self.path_last_fr = fr

        self.path_last_sl = sl
        self.path_last_sr = sr

        self.path_last_gz = gz


        self.reset_drive_controller()


        print()
        print("==================================================")
        print(" STOP -> MAX PATH SAMPLE")
        print("==================================================")


        print(
            f"FL={fl:.3f} "
            f"FR={fr:.3f}"
        )


        print(
            f"SL={sl:.3f} "
            f"SR={sr:.3f}"
        )


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


        if abs(gz) <= MAX_PATH_GYRO_LIMIT:

            self.path_fl_sum += self.clean_sensor(
                fl,
                MAX_PATH_SENSOR_CAP
            )


            self.path_fr_sum += self.clean_sensor(
                fr,
                MAX_PATH_SENSOR_CAP
            )


            self.path_sl_sum += self.clean_sensor(
                sl,
                MAX_PATH_SENSOR_CAP
            )


            self.path_sr_sum += self.clean_sensor(
                sr,
                MAX_PATH_SENSOR_CAP
            )


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


        if not (
            enough
            or
            timeout
        ):

            return (
                0.0,
                0.0
            )


        # ========================================================
        # AVERAGE
        # ========================================================

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
            reason
        )


    # ============================================================
    # START TURN
    # ============================================================

    def start_turn(
        self,
        direction,
        reason
    ):

        self.turn_dir = direction

        self.mode = "BRAKE_TURN"

        self.command_speed = 0.0

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
        print(" 80 DEG TURN")
        print("==================================================")


        print(
            f"TURN = {side}"
        )


        print(
            f"Reason = {reason}"
        )


        print(
            "TURN CONTROLLER UNCHANGED"
        )


        print("==================================================")
        print()


        return (
            0.0,
            0.0
        )


    # ============================================================
    # TURN BRAKE
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
    # UNCHANGED
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
        # LOCKED
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
                f"{turned:.2f}",
                flush=True
            )


            self.mode = "TURN_STOP"

            self.turn_stop_time = 0.0


            return (
                0.0,
                0.0
            )


        # ========================================================
        # OVERSHOOT
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


            if turned >= TURN_RESCUE_ANGLE:

                command = max(
                    command,
                    0.65
                )


        # ========================================================
        # NORMAL TURN
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


            if error > 25.0:

                command = self.clamp(
                    raw_command,
                    1.10,
                    TURN_MAX_SPEED
                )


            elif error > 12.0:

                command = self.clamp(
                    raw_command,
                    0.65,
                    1.55
                )


            elif error > 5.0:

                command = self.clamp(
                    raw_command,
                    0.30,
                    0.90
                )


            elif error > TURN_TOLERANCE:

                command = self.clamp(
                    raw_command,
                    0.0,
                    0.45
                )


                if (
                    command < 0.12

                    and

                    rate_abs < 4.0

                    and

                    error > 1.0
                ):

                    command = 0.15


            else:

                command = 0.0


        # ========================================================
        # HARD PROTECTION
        # ========================================================

        if turned >= TURN_HARD_LIMIT:

            desired_dir = (
                -self.turn_dir
            )

            command = 0.75


        # ========================================================
        # MOTOR
        # ========================================================

        if desired_dir > 0:

            left = -command

            right = +command


        else:

            left = +command

            right = -command


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

            self.command_speed = 0.0


            self.reset_drive_controller()


        return (
            0.0,
            0.0
        )


    # ============================================================
    # STEP
    # ============================================================

    def step(
        self,
        data
    ):

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
        # STATE
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
        # ========================================================

        if self.mode == "SETTLE":

            self.settle_time += dt


            predicted_sl, predicted_sr = self.predict_sides(
                sl,
                sr,
                dt
            )


            desired_speed = self.choose_drive_speed(
                sl,
                sr,
                predicted_sl,
                predicted_sr,
                dt,
                True
            )


            desired_speed = min(
                desired_speed,
                SETTLE_SPEED
            )


            (
                left,
                right,
                drive_mode,
                error,
                front_mode,
                correction,
                actual_speed
            ) = self.straight_drive(
                sl,
                sr,
                fl,
                fr,
                gz,
                dt,
                desired_speed
            )


            if self.settle_time >= SETTLE_TIME:

                self.mode = "DRIVE"

                self.completed_turns += 1


                self.turn_lockout_until = (
                    self.sim_time
                    +
                    POST_TURN_LOCKOUT
                )


                self.side_detector_armed = False

                self.corner_timer = 0.0


                self.reset_drive_controller()


                print(
                    "TURN COMPLETE -> DRIVE",
                    flush=True
                )


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
        # ========================================================

        if self.completed_turns == 0:

            navigation_mode = "FIRST"


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
                        gz
                    )


            predicted_sl, predicted_sr = self.predict_sides(
                sl,
                sr,
                dt
            )


            desired_speed = self.choose_drive_speed(
                sl,
                sr,
                predicted_sl,
                predicted_sr,
                dt,
                False
            )


            (
                left,
                right,
                drive_mode,
                error,
                front_mode,
                correction,
                actual_speed
            ) = self.straight_drive(
                sl,
                sr,
                fl,
                fr,
                gz,
                dt,
                desired_speed
            )


        # ========================================================
        # AFTER FIRST TURN
        # ========================================================

        else:

            navigation_mode = "FRONT015"


            # ====================================================
            # CORNER ARM
            # ====================================================

            if not self.side_detector_armed:

                if (
                    sl > SIDE_ARM_DIST

                    and

                    sr > SIDE_ARM_DIST
                ):

                    self.side_detector_armed = True

                    self.corner_timer = 0.0


            # ====================================================
            # CORNER CONDITION
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

                    print(
                        "CORNER CONFIRMED",
                        flush=True
                    )


                    return self.start_path_sample(
                        raw_sl,
                        raw_sr,
                        raw_fl,
                        raw_fr,
                        gz
                    )


            else:

                self.corner_timer = 0.0


            # ====================================================
            # PREDICT
            # ====================================================

            predicted_sl, predicted_sr = self.predict_sides(
                sl,
                sr,
                dt
            )


            # ====================================================
            # SPEED
            # ====================================================

            desired_speed = self.choose_drive_speed(
                sl,
                sr,
                predicted_sl,
                predicted_sr,
                dt,
                lockout_active
            )


            # ====================================================
            # DRIVE
            # ====================================================

            (
                left,
                right,
                drive_mode,
                error,
                front_mode,
                correction,
                actual_speed
            ) = self.straight_drive(
                sl,
                sr,
                fl,
                fr,
                gz,
                dt,
                desired_speed
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
            0.08
        ):

            self.last_log = now


            difference = (
                sr
                -
                sl
            )


            print(
                f"DRIVE "
                f"{drive_mode:6s} "
                f"mode={navigation_mode} "
                f"FL={fl:.3f} "
                f"FR={fr:.3f} "
                f"SL={sl:.3f} "
                f"SR={sr:.3f} "
                f"DIFF={difference:+.3f} "
                f"speed={actual_speed:.2f} "
                f"front={front_mode} "
                f"corr={correction:+.2f} "
                f"predSL={predicted_sl:.3f} "
                f"predSR={predicted_sr:.3f} "
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
    print(" PACBOT BEST CONTROLLER + WALL DISTANCE FIX")
    print("==================================================")
    print()

    print("ONLY CHANGE")
    print("------------------------------------")

    print(
        "SIDE_HARD_DIST:"
    )

    print(
        "OLD = 0.11"
    )

    print(
        "NEW = 0.14"
    )

    print(
        "+0.03 distance"
    )

    print()

    print("FRONT CONTROL")
    print("------------------------------------")

    print(
        "FL < .15 -> RIGHT"
    )

    print(
        "FR < .15 -> LEFT"
    )

    print(
        "both > .15 -> normal PID"
    )

    print()

    print("TURN")
    print("------------------------------------")

    print(
        f"target = "
        f"{TURN_TARGET} deg"
    )

    print(
        "TURN LOGIC UNCHANGED"
    )

    print()

    print("SPEED")
    print("------------------------------------")

    print(
        f"MAX = "
        f"{BASE_SPEED}"
    )

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
