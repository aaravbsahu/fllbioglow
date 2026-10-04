"""Deterministic replay, generated from 1_run_20261004_110715.csv.

Hand-edit the numbers in PROGRAM below to fine-tune -- each tuple is one
step, run in order:
  ("drive", encoder_degrees)   -- straight, holding heading via gyro
  ("turn", degrees)            -- relative pivot turn, gyro-corrected
  ("arm", right_angle, left_angle)  -- move arms to these angles
    (relative to wherever the arms were at program start)
Delete a step, change a number, add a new one -- it's just a list.
"""
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Button
from pybricks.tools import wait, StopWatch

hub = PrimeHub()
hub.system.set_stop_button(None)
hub.imu.reset_heading(0)

left_motor = Motor(Port.F)
right_motor = Motor(Port.E)
right_arm_motor = Motor(Port.C)
left_arm_motor = Motor(Port.D)

CRUISE = 600        # deg/s wheel speed while driving straight
KP_HEAD = 9.0        # steering gain, both drive() and turn()
MAX_CMD = 950        # deg/s hard cap per wheel
DRIVE_TOL = 15       # encoder-deg: close enough to end a drive() leg
TURN_TOL = 4         # deg: close enough to end a turn() leg
ARM_TOL = 8          # motor-deg: close enough to end an arm_to() step
STEP_TIMEOUT_MS = 6000
WATCHDOG_MS = 180000

watchdog = StopWatch()
press_count = 0
was_pressed = False
aborted = False


def dist_now():
    return (right_motor.angle() - left_motor.angle()) / 2


def clamp(v, lim):
    return lim if v > lim else (-lim if v < -lim else v)


def check_abort():
    global press_count, was_pressed, aborted
    pressed = Button.CENTER in hub.buttons.pressed()
    if pressed and not was_pressed:
        press_count += 1
    was_pressed = pressed
    if press_count >= 3:
        aborted = True
    return aborted


def drive(distance):
    """Drive `distance` encoder-degrees straight, holding the heading we
    had when this step started."""
    target_heading = hub.imu.heading()
    start = dist_now()
    target = start + distance
    timer = StopWatch()
    while not (aborted or watchdog.time() > WATCHDOG_MS):
        if check_abort():
            break
        err = target - dist_now()
        if abs(err) <= DRIVE_TOL or timer.time() > STEP_TIMEOUT_MS:
            break
        drive_cmd = clamp(3.0 * err, CRUISE)
        turn_cmd = KP_HEAD * (hub.imu.heading() - target_heading)
        left_motor.run(clamp(-drive_cmd + turn_cmd, MAX_CMD))
        right_motor.run(clamp(drive_cmd + turn_cmd, MAX_CMD))
        wait(20)
    left_motor.stop()
    right_motor.stop()


def turn(angle):
    """Pivot turn by `angle` degrees relative to the current heading,
    using the gyro to stop exactly there."""
    target_heading = hub.imu.heading() + angle
    timer = StopWatch()
    while not (aborted or watchdog.time() > WATCHDOG_MS):
        if check_abort():
            break
        err = hub.imu.heading() - target_heading
        if abs(err) <= TURN_TOL or timer.time() > STEP_TIMEOUT_MS:
            break
        turn_cmd = clamp(KP_HEAD * err, MAX_CMD)
        left_motor.run(turn_cmd)
        right_motor.run(turn_cmd)
        wait(20)
    left_motor.stop()
    right_motor.stop()


right_arm_zero = right_arm_motor.angle()
left_arm_zero = left_arm_motor.angle()


def arm_to(right_angle, left_angle):
    """Move both arms to the given angles, relative to their position
    when this program started."""
    timer = StopWatch()
    while not (aborted or watchdog.time() > WATCHDOG_MS):
        if check_abort():
            break
        right_arm_motor.track_target(right_arm_zero + right_angle)
        left_arm_motor.track_target(left_arm_zero + left_angle)
        ra_now = right_arm_motor.angle() - right_arm_zero
        la_now = left_arm_motor.angle() - left_arm_zero
        reached = abs(ra_now - right_angle) <= ARM_TOL and abs(la_now - left_angle) <= ARM_TOL
        if reached or timer.time() > STEP_TIMEOUT_MS:
            break
        wait(20)


# PROGRAM: edit freely. Run in order, top to bottom.
PROGRAM = [
    ('drive', 652.0),
    ('arm', -6.0, -2.0),
    ('arm', -1.0, -2.0),
    ('arm', 41.0, -2.0),
    ('arm', 3.0, -2.0),
    ('arm', -14.0, -2.0),
    ('arm', -42.0, -2.0),
    ('arm', -74.0, -2.0),
    ('arm', -64.0, -2.0),
    ('arm', 8.0, -2.0),
    ('arm', 15.0, -2.0),
    ('arm', 30.0, -2.0),
    ('drive', 36.5),
    ('arm', 26.0, -2.0),
    ('arm', -1.0, -2.0),
    ('arm', -6.0, -2.0),
    ('arm', -19.0, -2.0),
    ('arm', -24.0, -2.0),
    ('arm', -41.0, -2.0),
    ('arm', -70.0, -2.0),
    ('arm', -87.0, -2.0),
    ('arm', -93.0, -2.0),
    ('arm', -87.0, -2.0),
    ('drive', -709.5),
]

for step in PROGRAM:
    if aborted or watchdog.time() > WATCHDOG_MS:
        break
    kind = step[0]
    if kind == "drive":
        drive(step[1])
    elif kind == "turn":
        turn(step[1])
    else:
        arm_to(step[1], step[2])

print("DETERMINISTIC_ABORTED" if aborted else "DETERMINISTIC_DONE")
