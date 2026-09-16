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

# Fast clear corridor
BASE_SPEED = 5.0

# After a turn
SETTLE_SPEED = 2.2

# During post-turn lockout
LOCKOUT_SPEED = 3.0

MAX_WHEEL = 6.0


# ================================================================
# FIRST TURN
# ================================================================

INSIDE_SIDE_DIST = 0.12

FIRST_FRONT_DIST = 0.14

FIRST_FRONT_DEBOUNCE = 4


# ================================================================
# FIRST TURN SCAN
# ================================================================

SCAN_LIMIT = 100.0
SCAN_MIN_ANGLE = 25.0

SCAN_SPEED = 1.20

SCAN_SENSOR_CAP = 1.0

SCAN_SCORE_ALPHA = 0.05


# Final first-turn heading
FIRST_FINAL_ANGLE = 90.0


# ================================================================
# FIRST TURN ALIGN
# ================================================================

ALIGN_TOLERANCE = 0.60

ALIGN_KP = 0.055
ALIGN_KD = 0.010

ALIGN_MAX_SPEED = 1.20
ALIGN_MIN_SPEED = 0.65

ALIGN_TIMEOUT = 4.0


# ================================================================
# AFTER FIRST TURN - REAL CORNER DETECTION
#
# A random single low side reading must NOT cause a turn.
#
# Real corner:
#
# min(SL, SR) <= 0.12
# AND
# max(SL, SR) <= 0.16
# ================================================================

SIDE_TURN_DIST = 0.12

SIDE_CONFIRM_DIST = 0.16

SIDE_ARM_DIST = 0.18

CORNER_CONFIRM_TIME = 0.045


# ================================================================
# MAX PATH SAMPLING
#
# When a real corner is found:
#
# STOP
# collect FL FR SL SR
# only accept samples when gyro is stable
# average samples
# choose MAX side
# ================================================================

MAX_PATH_SAMPLE_TIME = 0.10

MAX_PATH_SAMPLE_TIMEOUT = 0.30

MAX_PATH_SENSOR_CAP = 1.0

MAX_PATH_GYRO_LIMIT = 0.20


# Difference required to call one side clearly better.
MAX_PATH_MIN_DIFFERENCE = 0.04


# Front sensor gets slightly more importance.
MAX_PATH_FRONT_WEIGHT = 0.60
MAX_PATH_SIDE_WEIGHT = 0.40


# ================================================================
# FRONT COLLISION SAFETY
#
# After first turn:
#
# FL <= 0.10 -> steer RIGHT
# FR <= 0.10 -> steer LEFT
#
# BOTH <= 0.10 -> STOP / decide path
# ================================================================

FRONT_SAFE_DIST = 0.10

FRONT_AVOID_GAIN = 18.0

FRONT_AVOID_MIN = 0.50
FRONT_AVOID_MAX = 1.50

FRONT_AVOID_SPEED = 3.0


# ================================================================
# SIDE COLLISION SAFETY
#
# This is NOT a turn decision.
#
# It only pushes the robot away from a nearby wall.
# ================================================================

SIDE_SAFE_DIST = 0.11

SIDE_AVOID_GAIN = 16.0

SIDE_AVOID_MIN = 0.45
SIDE_AVOID_MAX = 1.40


# Absolute last safety distance.
SIDE_EMERGENCY_DIST = 0.075


# ================================================================
# ADAPTIVE SPEED
# ================================================================

SLOW_DIST_1 = 0.22
SLOW_DIST_2 = 0.18
SLOW_DIST_3 = 0.145

SPEED_1 = 4.2
SPEED_2 = 3.2
SPEED_3 = 2.2


# ================================================================
# POST TURN LOCKOUT
# ================================================================

POST_TURN_LOCKOUT = 1.5


# ================================================================
# WALL FOLLOW PID
# ================================================================

WALL_MIN = 0.025

WALL_MAX = 0.35

TARGET_WALL = 0.18


KP = 4.5
KI = 0.04
KD = 0.18

KG = 0.90


MAX_CORRECTION = 1.60

INTEGRAL_LIMIT = 0.10

ERROR_LIMIT = 0.10

ERROR_DEADBAND = 0.002

D_ALPHA = 0.03

SLEW_RATE = 10.0


# ================================================================
# EXACT 90 DEG TURN
# ================================================================

TURN_TARGET = 90.0

TURN_TOLERANCE = 0.60


TURN_KP = 0.045
TURN_KD = 0.010


TURN_MAX_SPEED = 1.80
TURN_MIN_SPEED = 0.72


TURN_FINAL_ZONE = 10.0
TURN_FINAL_MAX = 0.85


TURN_TIMEOUT = 7.0


# Hard safety
TURN_HARD_LIMIT = 95.0


# ================================================================
# TIMING
# ================================================================

BRAKE_TIME = 0.22

TURN_STOP_TIME = 0.18

SETTLE_TIME = 0.70


# ================================================================
# CONTROLLER
# ================================================================

class Controller:

    def __init__(self):

        # --------------------------------------------------------
        # STATES
        #
        # DRIVE
        # BRAKE_FIRST
        # FIRST_SCAN
        # FIRST_ALIGN
        # PATH_SAMPLE
        # BRAKE_90
        # TURN_90
        # TURN_STOP
        # SETTLE
        # --------------------------------------------------------

        self.mode = "DRIVE"


        # ========================================================
        # PID
        # ========================================================

        self.integral = 0.0

        self.previous_error = 0.0

        self.filtered_derivative = 0.0

        self.previous_correction = 0.0

        self.have_previous = False


        # ========================================================
        # MAZE
        # ========================================================

        self.inside_maze = False


        # ========================================================
        # TURN COUNT
        # ========================================================

        self.completed_turns = 0


        # ========================================================
        # FIRST TURN
        # ========================================================

        self.first_front_count = 0


        # ========================================================
        # CORNER DETECTOR
        # ========================================================

        self.side_detector_armed = False

        self.corner_timer = 0.0


        # ========================================================
        # TURN
        #
        # +1 = LEFT
        # -1 = RIGHT
        # ========================================================

        self.turn_dir = +1


        # ========================================================
        # GYRO
        # ========================================================

        self.gyro_bias = 0.0

        self.gyro_sum = 0.0

        self.gyro_samples = 0


        self.turn_angle = 0.0

        self.turn_time = 0.0


        # ========================================================
        # FIRST SCAN
        # ========================================================

        self.best_score = -1.0

        self.best_angle = 90.0


        self.scan_filtered_score = 0.0

        self.scan_score_ready = False


        # ========================================================
        # MAX PATH SAMPLING
        # ========================================================

        self.path_sample_time = 0.0

        self.path_total_time = 0.0

        self.path_sample_count = 0


        self.path_fl_sum = 0.0
        self.path_fr_sum = 0.0

        self.path_sl_sum = 0.0
        self.path_sr_sum = 0.0

        self.path_gz_sum = 0.0


        # Last measurements used as fallback.
        self.path_last_fl = 0.0
        self.path_last_fr = 0.0

        self.path_last_sl = 0.0
        self.path_last_sr = 0.0

        self.path_last_gz = 0.0


        # ========================================================
        # TIMERS
        # ========================================================

        self.brake_time = 0.0

        self.align_time = 0.0

        self.turn_stop_time = 0.0

        self.settle_time = 0.0


        self.sim_time = 0.0

        self.turn_lockout_until = 0.0


        # ========================================================
        # LOG
        # ========================================================

        self.last_log = 0.0


    # ============================================================
    # HELPERS
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
    def wall_visible(value):

        return (
            WALL_MIN
            <
            value
            <
            WALL_MAX
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


    # ============================================================
    # RESET PID
    # ============================================================

    def reset_pid(self):

        self.integral = 0.0

        self.previous_error = 0.0

        self.filtered_derivative = 0.0

        self.previous_correction = 0.0

        self.have_previous = False


    # ============================================================
    # RESET MAX PATH SAMPLES
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
    # FRONT COLLISION SAFETY
    # ============================================================

    def front_safety(
        self,
        fl,
        fr
    ):

        fl_close = (
            fl <= FRONT_SAFE_DIST
        )

        fr_close = (
            fr <= FRONT_SAFE_DIST
        )


        # ========================================================
        # BOTH TOO CLOSE
        # ========================================================

        if (
            fl_close
            and
            fr_close
        ):

            return (
                0.0,
                True,
                "BOTH_FRONT_CLOSE"
            )


        # ========================================================
        # FL TOO CLOSE
        #
        # Steer RIGHT
        # ========================================================

        if fl_close:

            distance_error = (
                FRONT_SAFE_DIST
                -
                fl
            )


            correction = (
                FRONT_AVOID_GAIN
                *
                distance_error
            )


            correction = self.clamp(
                correction,
                FRONT_AVOID_MIN,
                FRONT_AVOID_MAX
            )


            return (
                +correction,
                False,
                "FL_CLOSE_RIGHT"
            )


        # ========================================================
        # FR TOO CLOSE
        #
        # Steer LEFT
        # ========================================================

        if fr_close:

            distance_error = (
                FRONT_SAFE_DIST
                -
                fr
            )


            correction = (
                FRONT_AVOID_GAIN
                *
                distance_error
            )


            correction = self.clamp(
                correction,
                FRONT_AVOID_MIN,
                FRONT_AVOID_MAX
            )


            return (
                -correction,
                False,
                "FR_CLOSE_LEFT"
            )


        return (
            0.0,
            False,
            "CLEAR"
        )


    # ============================================================
    # SIDE COLLISION SAFETY
    # ============================================================

    def side_safety(
        self,
        sl,
        sr
    ):

        sl_close = (
            sl <= SIDE_SAFE_DIST
        )

        sr_close = (
            sr <= SIDE_SAFE_DIST
        )


        # Both close.
        if (
            sl_close
            and
            sr_close
        ):

            return (
                0.0,
                "BOTH_SIDE_CLOSE"
            )


        # ========================================================
        # LEFT SIDE CLOSE
        #
        # Push RIGHT
        # ========================================================

        if sl_close:

            error = (
                SIDE_SAFE_DIST
                -
                sl
            )


            correction = (
                SIDE_AVOID_GAIN
                *
                error
            )


            correction = self.clamp(
                correction,
                SIDE_AVOID_MIN,
                SIDE_AVOID_MAX
            )


            return (
                +correction,
                "SL_CLOSE_RIGHT"
            )


        # ========================================================
        # RIGHT SIDE CLOSE
        #
        # Push LEFT
        # ========================================================

        if sr_close:

            error = (
                SIDE_SAFE_DIST
                -
                sr
            )


            correction = (
                SIDE_AVOID_GAIN
                *
                error
            )


            correction = self.clamp(
                correction,
                SIDE_AVOID_MIN,
                SIDE_AVOID_MAX
            )


            return (
                -correction,
                "SR_CLOSE_LEFT"
            )


        return (
            0.0,
            "CLEAR"
        )


    # ============================================================
    # PID DRIVE
    # ============================================================

    def pid_drive(
        self,
        sl,
        sr,
        fl,
        fr,
        gz,
        dt,
        speed,
        use_collision_safety=True
    ):

        left_wall = self.wall_visible(
            sl
        )

        right_wall = self.wall_visible(
            sr
        )


        # ========================================================
        # BOTH SIDE WALLS
        # ========================================================

        if (
            left_wall
            and
            right_wall
        ):

            error = (
                sr
                -
                sl
            )

            drive_mode = "CENTER"


        # ========================================================
        # LEFT WALL ONLY
        # ========================================================

        elif left_wall:

            error = (
                TARGET_WALL
                -
                sl
            )

            drive_mode = "LEFT"


        # ========================================================
        # RIGHT WALL ONLY
        # ========================================================

        elif right_wall:

            error = (
                sr
                -
                TARGET_WALL
            )

            drive_mode = "RIGHT"


        # ========================================================
        # NO WALL
        # ========================================================

        else:

            error = 0.0

            drive_mode = "GYRO"


        # ========================================================
        # ERROR LIMIT
        # ========================================================

        if abs(error) < ERROR_DEADBAND:

            error = 0.0


        error = self.clamp(
            error,
            -ERROR_LIMIT,
            ERROR_LIMIT
        )


        # ========================================================
        # INTEGRAL
        # ========================================================

        if abs(error) < 0.05:

            self.integral += (
                error
                *
                dt
            )

        else:

            self.integral *= 0.97


        self.integral = self.clamp(
            self.integral,
            -INTEGRAL_LIMIT,
            INTEGRAL_LIMIT
        )


        # ========================================================
        # DERIVATIVE
        # ========================================================

        if (
            self.have_previous
            and
            dt > 0.0001
        ):

            raw_derivative = (

                error
                -
                self.previous_error

            ) / dt

        else:

            raw_derivative = 0.0


        self.filtered_derivative = (

            D_ALPHA
            *
            raw_derivative

            +

            (
                1.0
                -
                D_ALPHA
            )
            *
            self.filtered_derivative
        )


        self.previous_error = error

        self.have_previous = True


        # ========================================================
        # PID + GYRO
        # ========================================================

        correction = (

            KP
            *
            error

            +

            KI
            *
            self.integral

            +

            KD
            *
            self.filtered_derivative

            +

            KG
            *
            gz
        )


        front_mode = "OFF"
        side_mode = "OFF"


        # ========================================================
        # COLLISION SAFETY
        # ========================================================

        if use_collision_safety:

            (
                front_correction,
                front_stop,
                front_mode
            ) = self.front_safety(
                fl,
                fr
            )


            # ----------------------------------------------------
            # BOTH FRONT TOO CLOSE
            # ----------------------------------------------------

            if front_stop:

                return (
                    0.0,
                    0.0,
                    "FRONT_STOP",
                    error,
                    front_mode,
                    "STOP"
                )


            (
                side_correction,
                side_mode
            ) = self.side_safety(
                sl,
                sr
            )


            correction += (
                front_correction
                +
                side_correction
            )


            if front_mode != "CLEAR":

                speed = min(
                    speed,
                    FRONT_AVOID_SPEED
                )


            if side_mode != "CLEAR":

                speed = min(
                    speed,
                    2.8
                )


        # ========================================================
        # CORRECTION LIMIT
        # ========================================================

        correction = self.clamp(
            correction,
            -MAX_CORRECTION,
            MAX_CORRECTION
        )


        # ========================================================
        # SLEW
        # ========================================================

        max_change = (
            SLEW_RATE
            *
            dt
        )


        correction = (

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


        self.previous_correction = correction


        # ========================================================
        # WHEELS
        # ========================================================

        left = (
            speed
            +
            correction
        )


        right = (
            speed
            -
            correction
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
            error,
            front_mode,
            side_mode
        )


    # ============================================================
    # ADAPTIVE SPEED
    # ============================================================

    def get_safe_speed(
        self,
        sl,
        sr,
        lockout_active
    ):

        nearest = min(
            sl,
            sr
        )


        if nearest <= SLOW_DIST_3:

            speed = SPEED_3


        elif nearest <= SLOW_DIST_2:

            speed = SPEED_2


        elif nearest <= SLOW_DIST_1:

            speed = SPEED_1


        else:

            speed = BASE_SPEED


        if lockout_active:

            speed = min(
                speed,
                LOCKOUT_SPEED
            )


        return speed


    # ============================================================
    # CALCULATE LEFT / RIGHT PATH SCORES
    #
    # ALL 4 SENSORS
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


        # ========================================================
        # LEFT SIDE
        # ========================================================

        left_score = (

            MAX_PATH_FRONT_WEIGHT
            *
            flc

            +

            MAX_PATH_SIDE_WEIGHT
            *
            slc
        )


        # ========================================================
        # RIGHT SIDE
        # ========================================================

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
    # CHOOSE MAX PATH
    #
    # Uses averaged:
    #
    # FL
    # FR
    # SL
    # SR
    # GYRO
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


        difference = (
            left_score
            -
            right_score
        )


        print()
        print("==================================================")
        print(" MAX PATH DECISION")
        print("==================================================")

        print(
            f"FL avg = {flc:.3f}"
        )

        print(
            f"FR avg = {frc:.3f}"
        )

        print(
            f"SL avg = {slc:.3f}"
        )

        print(
            f"SR avg = {src:.3f}"
        )

        print(
            f"GZ avg = {gz:+.4f}"
        )

        print()

        print(
            f"LEFT SCORE  = "
            f"{left_score:.4f}"
        )

        print(
            f"RIGHT SCORE = "
            f"{right_score:.4f}"
        )

        print(
            f"DIFFERENCE  = "
            f"{difference:+.4f}"
        )

        print()


        # ========================================================
        # LEFT CLEARLY BIGGER
        # ========================================================

        if (
            left_score
            >
            right_score
            +
            MAX_PATH_MIN_DIFFERENCE
        ):

            print(
                "MAX PATH = LEFT"
            )

            print("==================================================")
            print()


            return (
                +1,
                (
                    f"MAX LEFT "
                    f"{left_score:.3f} > "
                    f"RIGHT {right_score:.3f}"
                )
            )


        # ========================================================
        # RIGHT CLEARLY BIGGER
        # ========================================================

        if (
            right_score
            >
            left_score
            +
            MAX_PATH_MIN_DIFFERENCE
        ):

            print(
                "MAX PATH = RIGHT"
            )

            print("==================================================")
            print()


            return (
                -1,
                (
                    f"MAX RIGHT "
                    f"{right_score:.3f} > "
                    f"LEFT {left_score:.3f}"
                )
            )


        # ========================================================
        # SCORES VERY CLOSE
        #
        # Use the currently working INVERTED side fallback:
        #
        # SL closer -> RIGHT
        # SR closer -> LEFT
        # ========================================================

        if slc < src:

            print(
                "MAX SCORES CLOSE"
            )

            print(
                "SL CLOSER -> RIGHT"
            )

            print("==================================================")
            print()


            return (
                -1,
                "MAX TIE -> SL CLOSER -> RIGHT"
            )


        else:

            print(
                "MAX SCORES CLOSE"
            )

            print(
                "SR CLOSER -> LEFT"
            )

            print("==================================================")
            print()


            return (
                +1,
                "MAX TIE -> SR CLOSER -> LEFT"
            )


    # ============================================================
    # FIRST TURN INITIAL DIRECTION
    #
    # Also uses ALL FOUR sensors.
    # ============================================================

    def choose_first_direction(
        self,
        fl,
        fr,
        sl,
        sr
    ):

        (
            left_score,
            right_score
        ) = self.calculate_path_scores(
            fl,
            fr,
            sl,
            sr
        )


        if left_score > right_score:

            return (
                +1,
                (
                    f"FIRST MAX LEFT "
                    f"{left_score:.3f} > "
                    f"{right_score:.3f}"
                )
            )


        return (
            -1,
            (
                f"FIRST MAX RIGHT "
                f"{right_score:.3f} > "
                f"{left_score:.3f}"
            )
        )


    # ============================================================
    # FIRST SCAN SCORE
    #
    # Uses ALL FOUR DISTANCE SENSORS.
    #
    # Gyro handles the angle.
    # ============================================================

    def first_scan_score(
        self,
        fl,
        fr,
        sl,
        sr
    ):

        flc = self.clean_sensor(
            fl,
            SCAN_SENSOR_CAP
        )

        frc = self.clean_sensor(
            fr,
            SCAN_SENSOR_CAP
        )

        slc = self.clean_sensor(
            sl,
            SCAN_SENSOR_CAP
        )

        src = self.clean_sensor(
            sr,
            SCAN_SENSOR_CAP
        )


        # --------------------------------------------------------
        # Prefer both front rays being open.
        # --------------------------------------------------------

        front_near = min(
            flc,
            frc
        )


        front_far = max(
            flc,
            frc
        )


        front_score = (

            0.70
            *
            front_near

            +

            0.30
            *
            front_far
        )


        # --------------------------------------------------------
        # Also use side space.
        # --------------------------------------------------------

        side_score = max(
            slc,
            src
        )


        return (

            0.80
            *
            front_score

            +

            0.20
            *
            side_score
        )


    # ============================================================
    # START FIRST SCAN
    # ============================================================

    def start_first_scan(
        self,
        sl,
        sr,
        fl,
        fr
    ):

        (
            self.turn_dir,
            reason
        ) = self.choose_first_direction(
            fl,
            fr,
            sl,
            sr
        )


        self.mode = "BRAKE_FIRST"


        self.brake_time = 0.0


        self.gyro_sum = 0.0
        self.gyro_samples = 0


        self.turn_angle = 0.0


        self.best_score = -1.0

        self.best_angle = 90.0


        self.scan_filtered_score = 0.0

        self.scan_score_ready = False


        self.first_front_count = 0


        self.reset_pid()


        direction = (
            "LEFT"
            if self.turn_dir > 0
            else
            "RIGHT"
        )


        print()
        print("==================================================")
        print(" FIRST TURN MAX SCAN")
        print("==================================================")

        print(
            f"FL={fl:.3f} "
            f"FR={fr:.3f}"
        )

        print(
            f"SL={sl:.3f} "
            f"SR={sr:.3f}"
        )

        print()

        print(
            f"Reason = {reason}"
        )

        print(
            f"SCAN = {direction}"
        )

        print()

        print(
            "FINAL HEADING WILL BE 90 DEG"
        )

        print("==================================================")
        print()


        return (
            0.0,
            0.0
        )


    # ============================================================
    # FIRST BRAKE
    # ============================================================

    def brake_first(
        self,
        gz,
        dt
    ):

        self.brake_time += dt


        self.gyro_sum += gz

        self.gyro_samples += 1


        if (
            self.brake_time
            >=
            BRAKE_TIME
        ):

            self.gyro_bias = (

                self.gyro_sum

                /

                max(
                    1,
                    self.gyro_samples
                )
            )


            self.turn_angle = 0.0


            self.mode = "FIRST_SCAN"


            print(
                "FIRST SCAN START",
                flush=True
            )


        return (
            0.0,
            0.0
        )


    # ============================================================
    # FIRST SCAN
    # ============================================================

    def first_scan(
        self,
        sl,
        sr,
        fl,
        fr,
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


        # Magnitude only.
        progress = abs(
            self.turn_angle
        )


        # ========================================================
        # ALL FOUR SENSOR SCORE
        # ========================================================

        raw_score = self.first_scan_score(
            fl,
            fr,
            sl,
            sr
        )


        if not self.scan_score_ready:

            self.scan_filtered_score = (
                raw_score
            )

            self.scan_score_ready = True


        else:

            self.scan_filtered_score = (

                SCAN_SCORE_ALPHA
                *
                raw_score

                +

                (
                    1.0
                    -
                    SCAN_SCORE_ALPHA
                )
                *
                self.scan_filtered_score
            )


        # ========================================================
        # SAVE MAX
        # ========================================================

        if (
            SCAN_MIN_ANGLE
            <=
            progress
            <=
            SCAN_LIMIT
        ):

            if (
                self.scan_filtered_score
                >
                self.best_score
            ):

                self.best_score = (
                    self.scan_filtered_score
                )

                self.best_angle = (
                    progress
                )


        # ========================================================
        # SCAN COMPLETE
        # ========================================================

        if progress >= SCAN_LIMIT:

            self.mode = "FIRST_ALIGN"

            self.align_time = 0.0


            print()
            print("==================================================")
            print(" FIRST MAX SCAN COMPLETE")
            print("==================================================")

            print(
                f"Best sensor angle = "
                f"{self.best_angle:.2f}"
            )

            print(
                f"Best score = "
                f"{self.best_score:.3f}"
            )

            print()

            print(
                "FINAL PHYSICAL HEADING = 90 DEG"
            )

            print("==================================================")
            print()


            return (
                0.0,
                0.0
            )


        # ========================================================
        # SCAN MOTOR
        # ========================================================

        if self.turn_dir > 0:

            left = -SCAN_SPEED

            right = +SCAN_SPEED


        else:

            left = +SCAN_SPEED

            right = -SCAN_SPEED


        # ========================================================
        # LOG
        # ========================================================

        now = time.monotonic()


        if (
            now
            -
            self.last_log
            >
            0.10
        ):

            self.last_log = now


            print(
                f"FIRST_SCAN "
                f"angle={progress:6.2f} "
                f"score={self.scan_filtered_score:.3f} "
                f"best={self.best_score:.3f} "
                f"bestAngle={self.best_angle:.2f}",
                flush=True
            )


        return (
            left,
            right
        )


    # ============================================================
    # FIRST ALIGN
    #
    # Scan may go to ~100.
    #
    # Final orientation comes back to EXACTLY 90.
    # ============================================================

    def first_align(
        self,
        gz,
        dt
    ):

        self.align_time += dt


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


        current_angle = abs(
            self.turn_angle
        )


        error = (
            FIRST_FINAL_ANGLE
            -
            current_angle
        )


        # ========================================================
        # FINISHED
        # ========================================================

        if (
            abs(error)
            <=
            ALIGN_TOLERANCE
        ):

            print()
            print("==================================================")
            print(" FIRST TURN COMPLETE")
            print("==================================================")

            print(
                f"Final angle = "
                f"{current_angle:.2f}"
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

        if (
            self.align_time
            >=
            ALIGN_TIMEOUT
        ):

            print(
                f"FIRST ALIGN TIMEOUT "
                f"angle={current_angle:.2f}",
                flush=True
            )


            self.mode = "TURN_STOP"

            self.turn_stop_time = 0.0


            return (
                0.0,
                0.0
            )


        # ========================================================
        # DIRECTION
        #
        # Below 90:
        # continue original direction.
        #
        # Above 90:
        # reverse back toward 90.
        # ========================================================

        if error > 0.0:

            direction = (
                self.turn_dir
            )

        else:

            direction = (
                -self.turn_dir
            )


        command = (

            ALIGN_KP
            *
            abs(error)

            -

            ALIGN_KD
            *
            abs(rate_deg)
        )


        command = self.clamp(
            command,
            ALIGN_MIN_SPEED,
            ALIGN_MAX_SPEED
        )


        if direction > 0:

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
    # REAL CORNER CONDITION
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
    # START MAX PATH SAMPLING
    #
    # STOP ROBOT while making the decision.
    #
    # This prevents collision while collecting data.
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


        self.reset_path_samples()


        self.path_last_fl = fl
        self.path_last_fr = fr

        self.path_last_sl = sl
        self.path_last_sr = sr

        self.path_last_gz = gz


        self.reset_pid()


        print()
        print("==================================================")
        print(" CORNER CONFIRMED")
        print(" STOPPING FOR MAX PATH MEASUREMENT")
        print("==================================================")

        print(
            f"FL={fl:.3f} "
            f"FR={fr:.3f}"
        )

        print(
            f"SL={sl:.3f} "
            f"SR={sr:.3f}"
        )

        print(
            f"GZ={gz:+.4f}"
        )

        print("==================================================")
        print()


        return (
            0.0,
            0.0
        )


    # ============================================================
    # MAX PATH SAMPLE STATE
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


        # Keep last values.
        self.path_last_fl = fl
        self.path_last_fr = fr

        self.path_last_sl = sl
        self.path_last_sr = sr

        self.path_last_gz = gz


        # ========================================================
        # ONLY USE SENSOR DATA WHEN BOT IS STABLE
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


        # ========================================================
        # ENOUGH GOOD DATA
        # ========================================================

        enough_samples = (

            self.path_sample_time
            >=
            MAX_PATH_SAMPLE_TIME

            and

            self.path_sample_count > 0
        )


        # ========================================================
        # TIMEOUT FALLBACK
        # ========================================================

        sample_timeout = (

            self.path_total_time
            >=
            MAX_PATH_SAMPLE_TIMEOUT
        )


        if (
            enough_samples
            or
            sample_timeout
        ):

            # ----------------------------------------------------
            # USE AVERAGE IF AVAILABLE
            # ----------------------------------------------------

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


            # ----------------------------------------------------
            # NO STABLE SAMPLE:
            # use most recent values.
            # ----------------------------------------------------

            else:

                avg_fl = self.path_last_fl
                avg_fr = self.path_last_fr

                avg_sl = self.path_last_sl
                avg_sr = self.path_last_sr

                avg_gz = self.path_last_gz


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


            # Current sensor values for logging.
            current_sl = sl
            current_sr = sr

            current_fl = fl
            current_fr = fr


            self.reset_path_samples()


            return self.start_90_turn(

                direction,
                reason,

                current_sl,
                current_sr,

                current_fl,
                current_fr
            )


        # ========================================================
        # LOG WHILE SAMPLING
        # ========================================================

        now = time.monotonic()


        if (
            now
            -
            self.last_log
            >
            0.05
        ):

            self.last_log = now


            print(
                f"MAX_SAMPLE "
                f"time={self.path_sample_time:.3f} "
                f"total={self.path_total_time:.3f} "
                f"n={self.path_sample_count:03d} "
                f"FL={fl:.3f} "
                f"FR={fr:.3f} "
                f"SL={sl:.3f} "
                f"SR={sr:.3f} "
                f"GZ={gz:+.4f}",
                flush=True
            )


        # IMPORTANT:
        # robot stays stopped during max decision.
        return (
            0.0,
            0.0
        )


    # ============================================================
    # START EXACT 90 DEG TURN
    # ============================================================

    def start_90_turn(
        self,
        direction,
        reason,
        sl,
        sr,
        fl,
        fr
    ):

        self.turn_dir = direction


        self.mode = "BRAKE_90"


        self.brake_time = 0.0


        self.turn_angle = 0.0

        self.turn_time = 0.0


        self.gyro_sum = 0.0

        self.gyro_samples = 0


        self.corner_timer = 0.0

        self.side_detector_armed = False


        self.reset_path_samples()

        self.reset_pid()


        side = (
            "LEFT"
            if direction > 0
            else
            "RIGHT"
        )


        print()
        print("==================================================")
        print(" MAX PATH SELECTED")
        print("==================================================")

        print(
            f"SL={sl:.3f} "
            f"SR={sr:.3f}"
        )

        print(
            f"FL={fl:.3f} "
            f"FR={fr:.3f}"
        )

        print()

        print(
            f"Reason = {reason}"
        )

        print(
            f"TURN = {side}"
        )

        print()

        print(
            "TURN TARGET = EXACT 90 DEG"
        )

        print("==================================================")
        print()


        return (
            0.0,
            0.0
        )


    # ============================================================
    # BRAKE BEFORE 90 DEG TURN
    # ============================================================

    def brake_90(
        self,
        gz,
        dt
    ):

        self.brake_time += dt


        self.gyro_sum += gz

        self.gyro_samples += 1


        if (
            self.brake_time
            >=
            BRAKE_TIME
        ):

            self.gyro_bias = (

                self.gyro_sum

                /

                max(
                    1,
                    self.gyro_samples
                )
            )


            self.turn_angle = 0.0

            self.turn_time = 0.0


            self.mode = "TURN_90"


            direction = (
                "LEFT"
                if self.turn_dir > 0
                else
                "RIGHT"
            )


            print(
                f"90 DEG {direction} TURN START",
                flush=True
            )


        return (
            0.0,
            0.0
        )


    # ============================================================
    # EXACT 90 DEG TURN
    #
    # IMPORTANT:
    #
    # Angle measurement uses ABSOLUTE gyro rotation.
    #
    # Therefore wrong gyro sign cannot cause a 360 degree spin.
    # ============================================================

    def turn_90(
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


        # ========================================================
        # PHYSICAL ROTATION MAGNITUDE
        # ========================================================

        turned = abs(
            self.turn_angle
        )


        remaining = (
            TURN_TARGET
            -
            turned
        )


        # ========================================================
        # 90 DEG COMPLETE
        # ========================================================

        if (
            remaining
            <=
            TURN_TOLERANCE
        ):

            print()
            print("==================================================")
            print(" 90 DEG TURN COMPLETE")
            print("==================================================")

            print(
                f"Turned = "
                f"{turned:.2f} DEG"
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
        # HARD STOP
        #
        # NO 180 / 270 / 360
        # ========================================================

        if turned >= TURN_HARD_LIMIT:

            print()
            print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            print(" HARD TURN LIMIT")
            print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

            print(
                f"Angle = {turned:.2f}"
            )

            print(
                "FORCING TURN STOP"
            )

            print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
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

        if (
            self.turn_time
            >=
            TURN_TIMEOUT
        ):

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
        # TURN CONTROL
        # ========================================================

        turn_rate = abs(
            rate_deg
        )


        command = (

            TURN_KP
            *
            remaining

            -

            TURN_KD
            *
            turn_rate
        )


        command = self.clamp(
            command,
            TURN_MIN_SPEED,
            TURN_MAX_SPEED
        )


        # ========================================================
        # SLOW NEAR 90
        # ========================================================

        if remaining < TURN_FINAL_ZONE:

            command = min(
                command,
                TURN_FINAL_MAX
            )


            command = max(
                command,
                TURN_MIN_SPEED
            )


        # ========================================================
        # PHYSICAL DIRECTION
        #
        # +1 = LEFT
        # -1 = RIGHT
        # ========================================================

        if self.turn_dir > 0:

            left = -command

            right = +command

            direction = "LEFT"


        else:

            left = +command

            right = -command

            direction = "RIGHT"


        # ========================================================
        # LOG
        # ========================================================

        now = time.monotonic()


        if (
            now
            -
            self.last_log
            >
            0.10
        ):

            self.last_log = now


            print(
                f"TURN {direction} "
                f"angle={turned:6.2f} "
                f"remain={remaining:6.2f} "
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


            self.reset_path_samples()

            self.reset_pid()


            print(
                "TURN STOPPED -> PID SETTLE",
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
        # SENSOR DATA
        # ========================================================

        sl = float(
            data["sl"]
        )

        sr = float(
            data["sr"]
        )

        fl = float(
            data["fl"]
        )

        fr = float(
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
        # STATES
        # ========================================================

        if self.mode == "BRAKE_FIRST":

            return self.brake_first(
                gz,
                dt
            )


        if self.mode == "FIRST_SCAN":

            return self.first_scan(
                sl,
                sr,
                fl,
                fr,
                gz,
                dt
            )


        if self.mode == "FIRST_ALIGN":

            return self.first_align(
                gz,
                dt
            )


        if self.mode == "PATH_SAMPLE":

            return self.path_sample(
                sl,
                sr,
                fl,
                fr,
                gz,
                dt
            )


        if self.mode == "BRAKE_90":

            return self.brake_90(
                gz,
                dt
            )


        if self.mode == "TURN_90":

            return self.turn_90(
                gz,
                dt
            )


        if self.mode == "TURN_STOP":

            return self.turn_stop(
                dt
            )


        # ========================================================
        # SETTLE AFTER TURN
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
            ) = self.pid_drive(

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


                self.reset_path_samples()

                self.reset_pid()


                print()
                print("==================================================")
                print(" TURN COMPLETE -> DRIVE")
                print("==================================================")

                print(
                    f"Completed turns = "
                    f"{self.completed_turns}"
                )

                print(
                    f"Lockout = "
                    f"{POST_TURN_LOCKOUT:.1f}s"
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
                sl <= INSIDE_SIDE_DIST

                or

                sr <= INSIDE_SIDE_DIST
            ):

                self.inside_maze = True

                self.first_front_count = 0


                print()
                print(
                    "BOT INSIDE MAZE",
                    flush=True
                )
                print()


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


        # ========================================================
        # DEFAULT LOG VARIABLE
        # ========================================================

        corner_now = False


        # ========================================================
        # FIRST TURN
        #
        # Keep original trigger.
        #
        # IMPORTANT:
        # front collision protection is NOT used here because
        # startup FL/FR readings may be small before maze entry.
        # ========================================================

        if self.completed_turns == 0:

            navigation_mode = "FIRST_MAX"


            if self.inside_maze:

                first_close = (

                    fl <= FIRST_FRONT_DIST

                    or

                    fr <= FIRST_FRONT_DIST
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

                    return self.start_first_scan(
                        sl,
                        sr,
                        fl,
                        fr
                    )


            forward_speed = BASE_SPEED


            # ----------------------------------------------------
            # FIRST-TURN DRIVE:
            #
            # Normal side PID only.
            # ----------------------------------------------------

            (
                left,
                right,
                drive_mode,
                error,
                front_mode,
                side_mode
            ) = self.pid_drive(

                sl,
                sr,
                fl,
                fr,
                gz,
                dt,

                forward_speed,

                False
            )


        # ========================================================
        # AFTER FIRST TURN
        # ========================================================

        else:

            navigation_mode = "MAX_4_SENSOR"


            # ====================================================
            # ARM CORNER DETECTOR
            #
            # Must leave previous corner before another turn.
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
                        "CORNER DETECTOR ARMED "
                        f"SL={sl:.3f} "
                        f"SR={sr:.3f}",
                        flush=True
                    )


            # ====================================================
            # CHECK REAL CORNER
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


                # =================================================
                # REAL CORNER CONFIRMED
                #
                # STOP AND SAMPLE ALL 4 SENSORS.
                # =================================================

                if (
                    self.corner_timer
                    >=
                    CORNER_CONFIRM_TIME
                ):

                    return self.start_path_sample(

                        sl,
                        sr,
                        fl,
                        fr,
                        gz
                    )


            else:

                self.corner_timer = 0.0


            # ====================================================
            # FRONT DEAD-END SAFETY
            #
            # BOTH front <=0.10.
            #
            # If lockout finished:
            # don't sit forever.
            #
            # Stop and make MAX decision.
            # ====================================================

            front_dead_end = (

                fl <= FRONT_SAFE_DIST

                and

                fr <= FRONT_SAFE_DIST
            )


            if front_dead_end:

                if not lockout_active:

                    return self.start_path_sample(

                        sl,
                        sr,
                        fl,
                        fr,
                        gz
                    )


            # ====================================================
            # EXTREME SIDE DANGER
            # ====================================================

            side_emergency = (

                sl <= SIDE_EMERGENCY_DIST

                or

                sr <= SIDE_EMERGENCY_DIST
            )


            if side_emergency:

                # ------------------------------------------------
                # During lockout:
                # STOP instead of collision.
                # ------------------------------------------------

                if lockout_active:

                    print(
                        f"EMERGENCY SAFETY STOP "
                        f"SL={sl:.3f} "
                        f"SR={sr:.3f}",
                        flush=True
                    )


                    return (
                        0.0,
                        0.0
                    )


                # ------------------------------------------------
                # Lockout over:
                # stop and sample MAX path.
                # ------------------------------------------------

                return self.start_path_sample(

                    sl,
                    sr,
                    fl,
                    fr,
                    gz
                )


            # ====================================================
            # SPEED
            # ====================================================

            forward_speed = self.get_safe_speed(
                sl,
                sr,
                lockout_active
            )


            # ====================================================
            # PID + COLLISION SAFETY
            # ====================================================

            (
                left,
                right,
                drive_mode,
                error,
                front_mode,
                side_mode
            ) = self.pid_drive(

                sl,
                sr,
                fl,
                fr,
                gz,
                dt,

                forward_speed,

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
            0.12
        ):

            self.last_log = now


            print(
                f"DRIVE "
                f"{drive_mode:6s} "
                f"mode={navigation_mode} "
                f"FL={fl:.3f} "
                f"FR={fr:.3f} "
                f"SL={sl:.3f} "
                f"SR={sr:.3f} "
                f"GZ={gz:+.3f} "
                f"speed={forward_speed:.2f} "
                f"front={front_mode} "
                f"side={side_mode} "
                f"armed={self.side_detector_armed} "
                f"corner={corner_now} "
                f"ct={self.corner_timer:.3f} "
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
# CONTROLLER INSTANCE
# ================================================================

controller = Controller()


# ================================================================
# MQTT MESSAGE
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


        # Safety stop on error.
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
    print(" PACBOT - MAX 4 SENSOR + EXACT 90")
    print("==================================================")
    print()
    print("CLEAR SPEED:")
    print()
    print(" 5.0")
    print()
    print("FIRST TURN:")
    print()
    print(" old trigger:")
    print(" FL <= 0.14 OR FR <= 0.14")
    print()
    print(" direction uses:")
    print(" FL + FR + SL + SR")
    print()
    print(" scan score also uses all 4 sensors")
    print()
    print(" final physical heading = 90 deg")
    print()
    print("AFTER FIRST TURN:")
    print()
    print(" confirmed corner only:")
    print()
    print(" min(SL,SR) <= 0.12")
    print(" AND")
    print(" max(SL,SR) <= 0.16")
    print()
    print(" then robot STOPS")
    print(" averages FL FR SL SR")
    print(" waits for stable gyro")
    print()
    print("LEFT MAX SCORE:")
    print(" 0.60*FL + 0.40*SL")
    print()
    print("RIGHT MAX SCORE:")
    print(" 0.60*FR + 0.40*SR")
    print()
    print("larger score wins")
    print()
    print("TURN:")
    print()
    print(" exact 90 deg only")
    print(" hard stop at 95 deg")
    print(" no 180 / 270 / 360")
    print()
    print("COLLISION SAFETY:")
    print()
    print(" FL <= 0.10 -> steer RIGHT")
    print(" FR <= 0.10 -> steer LEFT")
    print(" SL <= 0.11 -> steer RIGHT")
    print(" SR <= 0.11 -> steer LEFT")
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
