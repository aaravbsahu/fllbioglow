"""Deterministic replay, generated from run_20261010_101325.csv.

Hand-edit the numbers in PROGRAM below to fine-tune -- each tuple is one
step, run in order:
  ("drive", encoder_degrees, speed)   -- straight, holding heading via gyro
  ("turn", degrees, speed)            -- relative pivot turn, gyro-corrected
  ("arm", right_angle, left_angle, speed)  -- move arms to these angles
    (relative to wherever the arms were at program start)
`speed` is deg/s -- the wheel (drive/turn) or motor (arm) speed cap for
just that step. Delete a step, change a number, add a new one -- it's
just a list.
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

CRUISE = 600        # deg/s default wheel speed cap for drive() (per-step override via PROGRAM)
KP_HEAD = 9.0        # steering gain, both drive() and turn()
MAX_CMD = 950        # deg/s hard cap per wheel, and default turn() speed
ARM_SPEED = 200      # deg/s default arm motor speed (per-step override via PROGRAM)
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


def drive(distance, speed=CRUISE):
    """Drive `distance` encoder-degrees straight at up to `speed` deg/s,
    holding the heading we had when this step started."""
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
        drive_cmd = clamp(3.0 * err, speed)
        turn_cmd = KP_HEAD * (hub.imu.heading() - target_heading)
        left_motor.run(clamp(-drive_cmd + turn_cmd, MAX_CMD))
        right_motor.run(clamp(drive_cmd + turn_cmd, MAX_CMD))
        wait(20)
    left_motor.stop()
    right_motor.stop()


def turn(angle, speed=MAX_CMD):
    """Pivot turn by `angle` degrees relative to the current heading, at
    up to `speed` deg/s, using the gyro to stop exactly there."""
    target_heading = hub.imu.heading() + angle
    timer = StopWatch()
    while not (aborted or watchdog.time() > WATCHDOG_MS):
        if check_abort():
            break
        err = hub.imu.heading() - target_heading
        if abs(err) <= TURN_TOL or timer.time() > STEP_TIMEOUT_MS:
            break
        turn_cmd = clamp(KP_HEAD * err, speed)
        left_motor.run(turn_cmd)
        right_motor.run(turn_cmd)
        wait(20)
    left_motor.stop()
    right_motor.stop()


right_arm_zero = right_arm_motor.angle()
left_arm_zero = left_arm_motor.angle()


def arm_to(right_angle, left_angle, speed=ARM_SPEED):
    """Move both arms to the given angles at up to `speed` deg/s, relative
    to their position when this program started."""
    right_arm_motor.run_target(speed, right_arm_zero + right_angle, wait=False)
    left_arm_motor.run_target(speed, left_arm_zero + left_angle, wait=False)
    timer = StopWatch()
    while not (aborted or watchdog.time() > WATCHDOG_MS):
        if check_abort():
            break
        ra_now = right_arm_motor.angle() - right_arm_zero
        la_now = left_arm_motor.angle() - left_arm_zero
        reached = abs(ra_now - right_angle) <= ARM_TOL and abs(la_now - left_angle) <= ARM_TOL
        if reached or timer.time() > STEP_TIMEOUT_MS:
            break
        wait(20)
    right_arm_motor.hold()
    left_arm_motor.hold()


# PROGRAM: edit freely. Run in order, top to bottom.
PROGRAM = [
    ('drive', 1032.0, 600),
    ('drive', 547.0, 600),
    ('arm', -4.0, 0.0, 200),
    ('arm', -76.0, 0.0, 200),
    ('arm', -96.0, 0.0, 200),
    ('arm', -92.0, 0.0, 200),
    ('arm', -97.0, 0.0, 200),
    ('arm', -110.0, 0.0, 200),
    ('arm', -159.0, 0.0, 200),
    ('drive', -571.5, 600),
    ('turn', -7.0, 950),
    ('drive', -24.0, 600),
    ('arm', -154.0, 0.0, 200),
    ('turn', 125.8, 950),
    ('turn', -93.0, 950),
]

for step in PROGRAM:
    if aborted or watchdog.time() > WATCHDOG_MS:
        break
    kind = step[0]
    if kind == "drive":
        drive(step[1], step[2])
    elif kind == "turn":
        turn(step[1], step[2])
    else:
        arm_to(step[1], step[2], step[3])

print("DETERMINISTIC_ABORTED" if aborted else "DETERMINISTIC_DONE")
