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

BASE_SPEED = 5.0

SETTLE_SPEED = 2.2

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


# ================================================================
# IMPORTANT
#
# First turn always finishes at 90 degrees.
# Scan best angle DOES NOT become final heading.
# ================================================================

FIRST_FINAL_ANGLE = 90.0


# ================================================================
# FIRST ALIGN
# ================================================================

ALIGN_TOLERANCE = 0.60

ALIGN_KP = 0.055
ALIGN_KD = 0.010

ALIGN_MAX_SPEED = 1.20
ALIGN_MIN_SPEED = 0.65

ALIGN_TIMEOUT = 4.0


# ================================================================
# AFTER FIRST TURN - CORNER DETECTION
#
# Do not turn from one side sensor alone.
#
# Real corner:
#
# nearest side <= 0.12
# AND
# farther side <= 0.16
# ================================================================

SIDE_TURN_DIST = 0.12

SIDE_CONFIRM_DIST = 0.16

SIDE_ARM_DIST = 0.18

CORNER_CONFIRM_TIME = 0.045


# ================================================================
# TURN DIRECTION
#
# First use FL/FR openness.
#
# Fallback:
#
# SL closer -> RIGHT
# SR closer -> LEFT
# ================================================================

DIRECTION_MARGIN = 0.06

DIRECTION_SENSOR_CAP = 1.0


# ================================================================
# FRONT COLLISION SAFETY
#
# FL <= 0.10 -> move RIGHT
#
# FR <= 0.10 -> move LEFT
#
# BOTH <= 0.10 -> STOP
# ================================================================

FRONT_SAFE_DIST = 0.10

FRONT_AVOID_GAIN = 18.0

FRONT_AVOID_MIN = 0.50

FRONT_AVOID_MAX = 1.50

FRONT_AVOID_SPEED = 3.0


# ================================================================
# SIDE COLLISION SAFETY
# ================================================================

SIDE_SAFE_DIST = 0.11

SIDE_AVOID_GAIN = 16.0

SIDE_AVOID_MIN = 0.45

SIDE_AVOID_MAX = 1.40


# Absolute danger.
SIDE_EMERGENCY_DIST = 0.075


# ================================================================
# SPEED CONTROL
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
# WALL FOLLOWING PID
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
# 90 DEG TURN
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


# ================================================================
# HARD ANGLE SAFETY
#
# Even if gyro/control becomes weird,
# never intentionally continue past 95 degrees.
# ================================================================

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
        # CORNER
        # ========================================================

        self.side_detector_armed = False

        self.corner_timer = 0.0


        # ========================================================
        # TURN DIRECTION
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
        # TIMERS
        # ========================================================

        self.brake_time = 0.0

        self.align_time = 0.0

        self.turn_stop_time = 0.0

        self.settle_time = 0.0


        self.sim_time = 0.0

        self.turn_lockout_until = 0.0


        # ========================================================
        # LOGGING
        # ========================================================

        self.last_log = 0.0


    # ============================================================
    # HELPER
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


    # ============================================================
    # WALL VISIBLE
    # ============================================================

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
    # CLEAN SENSOR
    # ============================================================

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
    # FRONT SAFETY
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


        # --------------------------------------------------------
        # BOTH CLOSE
        # --------------------------------------------------------

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


        # --------------------------------------------------------
        # FL CLOSE
        #
        # Move RIGHT
        # --------------------------------------------------------

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


        # --------------------------------------------------------
        # FR CLOSE
        #
        # Move LEFT
        # --------------------------------------------------------

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
    # SIDE SAFETY
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


        # --------------------------------------------------------
        # LEFT SIDE TOO CLOSE
        #
        # Move RIGHT
        # --------------------------------------------------------

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


        # --------------------------------------------------------
        # RIGHT SIDE TOO CLOSE
        #
        # Move LEFT
        # --------------------------------------------------------

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
    # STRAIGHT PID
    # ============================================================

    def pid_drive(
        self,
        sl,
        sr,
        fl,
        fr,
        gz,
        dt,
        speed
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
        # LEFT WALL
        # ========================================================

        elif left_wall:

            error = (
                TARGET_WALL
                -
                sl
            )

            drive_mode = "LEFT"


        # ========================================================
        # RIGHT WALL
        # ========================================================

        elif right_wall:

            error = (
                sr
                -
                TARGET_WALL
            )

            drive_mode = "RIGHT"


        # ========================================================
        # NO SIDE WALL
        # ========================================================

        else:

            error = 0.0

            drive_mode = "GYRO"


        # ========================================================
        # DEADBAND
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
        # NORMAL PID
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


        # ========================================================
        # FRONT SAFETY
        # ========================================================

        (
            front_correction,
            front_stop,
            front_mode
        ) = self.front_safety(
            fl,
            fr
        )


        # ========================================================
        # BOTH FRONT SENSORS TOO CLOSE
        # ========================================================

        if front_stop:

            return (
                0.0,
                0.0,
                "FRONT_STOP",
                error,
                front_mode,
                "STOP"
            )


        # ========================================================
        # SIDE SAFETY
        # ========================================================

        (
            side_correction,
            side_mode
        ) = self.side_safety(
            sl,
            sr
        )


        # ========================================================
        # COMBINE
        # ========================================================

        correction += (
            front_correction
            +
            side_correction
        )


        correction = self.clamp(
            correction,
            -MAX_CORRECTION,
            MAX_CORRECTION
        )


        # ========================================================
        # SAFETY SPEED REDUCTION
        # ========================================================

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
        # SLEW LIMITER
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
        # WHEEL SPEED
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
    # SAFE SPEED
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
    # FIRST SCAN DIRECTION
    # ============================================================

    def choose_first_direction(
        self,
        fl,
        fr
    ):

        flc = self.clean_sensor(
            fl,
            DIRECTION_SENSOR_CAP
        )


        frc = self.clean_sensor(
            fr,
            DIRECTION_SENSOR_CAP
        )


        if flc > frc:

            return (
                +1,
                "LEFT FRONT MORE OPEN -> LEFT"
            )


        return (
            -1,
            "RIGHT FRONT MORE OPEN -> RIGHT"
        )


    # ============================================================
    # FIRST SCAN SCORE
    #
    # Prefer BOTH beams being open.
    # ============================================================

    def first_scan_score(
        self,
        fl,
        fr
    ):

        flc = self.clean_sensor(
            fl,
            SCAN_SENSOR_CAP
        )


        frc = self.clean_sensor(
            fr,
            SCAN_SENSOR_CAP
        )


        near = min(
            flc,
            frc
        )


        far = max(
            flc,
            frc
        )


        return (

            0.75
            *
            near

            +

            0.25
            *
            far
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
            fr
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


        print()
        print("==================================================")
        print(" FIRST TURN SCAN")
        print("==================================================")
        print(
            f"FL={fl:.3f} "
            f"FR={fr:.3f} "
            f"SL={sl:.3f} "
            f"SR={sr:.3f}"
        )
        print(
            reason
        )
        print()
        print(
            "FINAL HEADING = EXACTLY 90 DEG"
        )
        print("==================================================")
        print()


        return (
            0.0,
            0.0
        )


    # ============================================================
    # BRAKE FIRST
    # ============================================================

    def brake_first(
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


        # IMPORTANT:
        # use absolute physical rotation.
        progress = abs(
            self.turn_angle
        )


        raw_score = self.first_scan_score(
            fl,
            fr
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
        # SAVE BEST FOR INFORMATION ONLY
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


                self.best_angle = progress


        # ========================================================
        # SCAN COMPLETE
        # ========================================================

        if progress >= SCAN_LIMIT:

            self.mode = "FIRST_ALIGN"

            self.align_time = 0.0


            print()
            print("==================================================")
            print(" FIRST SCAN COMPLETE")
            print("==================================================")
            print(
                f"Scan best = "
                f"{self.best_angle:.2f}"
            )
            print()
            print(
                "FINAL TURN TARGET = 90 DEG"
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
                f"best={self.best_score:.3f}",
                flush=True
            )


        return (
            left,
            right
        )


    # ============================================================
    # FIRST ALIGN
    #
    # Always align to physical 90 degrees.
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
        # EXACT 90
        # ========================================================

        if abs(error) <= ALIGN_TOLERANCE:

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
        # HARD LIMIT
        #
        # First-turn align also cannot spin forever.
        # ========================================================

        if current_angle >= TURN_HARD_LIMIT:

            print(
                f"FIRST TURN HARD LIMIT "
                f"{current_angle:.2f}",
                flush=True
            )


            self.mode = "TURN_STOP"

            self.turn_stop_time = 0.0


            return (
                0.0,
                0.0
            )


        if self.align_time >= ALIGN_TIMEOUT:

            print(
                f"FIRST ALIGN TIMEOUT "
                f"{current_angle:.2f}",
                flush=True
            )


            self.mode = "TURN_STOP"

            self.turn_stop_time = 0.0


            return (
                0.0,
                0.0
            )


        # ========================================================
        # Need more angle
        # ========================================================

        if error > 0.0:

            direction = self.turn_dir


        # ========================================================
        # Went beyond 90 during scan.
        #
        # Rotate backwards toward 90.
        # ========================================================

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
    # CHOOSE TURN DIRECTION
    # ============================================================

    def choose_corner_direction(
        self,
        fl,
        fr,
        sl,
        sr
    ):

        flc = self.clean_sensor(
            fl,
            DIRECTION_SENSOR_CAP
        )


        frc = self.clean_sensor(
            fr,
            DIRECTION_SENSOR_CAP
        )


        # ========================================================
        # FL more open -> LEFT
        # ========================================================

        if (
            flc
            >
            frc + DIRECTION_MARGIN
        ):

            return (
                +1,
                "FL MORE OPEN -> LEFT"
            )


        # ========================================================
        # FR more open -> RIGHT
        # ========================================================

        if (
            frc
            >
            flc + DIRECTION_MARGIN
        ):

            return (
                -1,
                "FR MORE OPEN -> RIGHT"
            )


        # ========================================================
        # FALLBACK
        #
        # INVERTED SIDE MAPPING
        #
        # SL closer -> RIGHT
        # SR closer -> LEFT
        # ========================================================

        if sl < sr:

            return (
                -1,
                "SL CLOSER -> RIGHT"
            )


        return (
            +1,
            "SR CLOSER -> LEFT"
        )


    # ============================================================
    # START NORMAL TURN
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


        self.reset_pid()


        side = (
            "LEFT"
            if direction > 0
            else
            "RIGHT"
        )


        print()
        print("==================================================")
        print(" CONFIRMED 90 DEG TURN")
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
            "TARGET = 90 DEG ONLY"
        )
        print("==================================================")
        print()


        return (
            0.0,
            0.0
        )


    # ============================================================
    # BRAKE BEFORE NORMAL TURN
    # ============================================================

    def brake_90(
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


            self.turn_angle = 0.0

            self.turn_time = 0.0


            self.mode = "TURN_90"


            print(
                "90 TURN START",
                flush=True
            )


        return (
            0.0,
            0.0
        )


    # ============================================================
    # NORMAL TURN
    #
    # CRITICAL FIX:
    #
    # turned = abs(self.turn_angle)
    #
    # Therefore wrong gyro sign cannot make it rotate forever.
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
        # ABSOLUTE PHYSICAL ROTATION
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

        if remaining <= TURN_TOLERANCE:

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
        # HARD 95 DEG STOP
        #
        # NEVER KEEP ROTATING TO 180 / 270 / 360
        # ========================================================

        if turned >= TURN_HARD_LIMIT:

            print()
            print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            print(" HARD TURN ANGLE LIMIT")
            print(
                f"ANGLE = {turned:.2f}"
            )
            print("STOPPING TURN")
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
        # TURN SPEED
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
        # SLOW DOWN NEAR 90
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
        # DIRECTION
        #
        # Only motor direction uses turn_dir.
        #
        # Angle measurement does NOT use turn_dir.
        # ========================================================

        if self.turn_dir > 0:

            # LEFT

            left = -command

            right = +command

            direction = "LEFT"


        else:

            # RIGHT

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
        # SENSORS
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
        # SETTLE
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
                SETTLE_SPEED
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


                self.reset_pid()


                print()
                print("==================================================")
                print(" TURN COMPLETE -> DRIVE")
                print("==================================================")
                print(
                    f"Turns = "
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


        # ========================================================
        # DEFAULT FOR LOG
        # ========================================================

        corner_now = False


        # ========================================================
        # FIRST TURN
        # ========================================================

        if self.completed_turns == 0:

            navigation_mode = "FIRST_TURN"


            if self.inside_maze:

                front_close = (

                    fl <= FIRST_FRONT_DIST

                    or

                    fr <= FIRST_FRONT_DIST
                )


                if front_close:

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


        # ========================================================
        # AFTER FIRST TURN
        # ========================================================

        else:

            navigation_mode = "CORNER_90_ONLY"


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
                        "CORNER DETECTOR ARMED "
                        f"SL={sl:.3f} "
                        f"SR={sr:.3f}",
                        flush=True
                    )


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


                # =================================================
                # CONFIRMED CORNER
                # =================================================

                if (
                    self.corner_timer
                    >=
                    CORNER_CONFIRM_TIME
                ):

                    (
                        direction,
                        reason
                    ) = self.choose_corner_direction(
                        fl,
                        fr,
                        sl,
                        sr
                    )


                    return self.start_90_turn(

                        direction,
                        reason,

                        sl,
                        sr,
                        fl,
                        fr
                    )


            else:

                self.corner_timer = 0.0


            # ====================================================
            # EMERGENCY SIDE DISTANCE
            # ====================================================

            if (
                sl <= SIDE_EMERGENCY_DIST

                or

                sr <= SIDE_EMERGENCY_DIST
            ):

                # ------------------------------------------------
                # During lockout:
                # STOP.
                # ------------------------------------------------

                if lockout_active:

                    print(
                        f"EMERGENCY STOP "
                        f"SL={sl:.3f} "
                        f"SR={sr:.3f}",
                        flush=True
                    )


                    return (
                        0.0,
                        0.0
                    )


                # ------------------------------------------------
                # Lockout done:
                # exact 90 degree escape turn.
                # ------------------------------------------------

                (
                    direction,
                    reason
                ) = self.choose_corner_direction(
                    fl,
                    fr,
                    sl,
                    sr
                )


                return self.start_90_turn(

                    direction,

                    "EMERGENCY " + reason,

                    sl,
                    sr,
                    fl,
                    fr
                )


            # ====================================================
            # SPEED
            # ====================================================

            forward_speed = self.get_safe_speed(
                sl,
                sr,
                lockout_active
            )


        # ========================================================
        # STRAIGHT DRIVE
        # ========================================================

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
            forward_speed
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
    print(" PACBOT - EXACT 90 DEG CONTROLLER")
    print("==================================================")
    print()
    print("SPEED:")
    print()
    print(" CLEAR = 5.0")
    print()
    print("FIRST TURN:")
    print()
    print(" FL <= 0.14 OR FR <= 0.14")
    print(" scan direction")
    print(" final heading = EXACT 90 DEG")
    print()
    print("AFTER FIRST TURN:")
    print()
    print(" confirmed corners only")
    print()
    print(" min(SL,SR) <= 0.12")
    print(" AND")
    print(" max(SL,SR) <= 0.16")
    print()
    print("TURN:")
    print()
    print(" ALWAYS 90 DEG")
    print(" HARD STOP AT 95 DEG")
    print()
    print("NO 180 / 270 / 360 TURN")
    print()
    print("COLLISION SAFETY:")
    print()
    print(" FL <= 0.10 -> steer RIGHT")
    print(" FR <= 0.10 -> steer LEFT")
    print()
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
