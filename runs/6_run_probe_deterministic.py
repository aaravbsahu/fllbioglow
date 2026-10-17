"""Deterministic replay, generated from probe.

Hand-edit the numbers in PROGRAM below to fine-tune -- each tuple is one
step, run in order:
  ("drive", encoder_degrees, speed)   -- straight, holding heading via gyro
  ("turn", degrees, speed)            -- relative pivot turn, gyro-corrected
  ("arm", right_angle, left_angle, speed)  -- move arms to these angles
    (relative to wherever the arms were at program start, or since the
    last relax_arm() of that arm)
  ("relax_arm", "right"|"left"|"both")  -- coast that arm motor so it can
    be pushed freely by hand, and make wherever it ends up the new zero
    for later "arm" steps on that motor
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
STEP_TIMEOUT_MS = 4000   # hard cap per step
STALL_MS = 800           # give up on a drive/turn that makes no progress this long
STALL_PROGRESS_DRIVE = 6 # encoder-deg of improvement that counts as progress
STALL_PROGRESS_TURN = 1.5 # deg of improvement that counts as progress
WATCHDOG_MS = 180000
RELAX_SETTLE_TOL = 3        # motor-deg: movement below this counts as "still"
RELAX_QUIET_MS = 300        # must stay still this long to count as settled
RELAX_TICK_MS = 50
RELAX_TIMEOUT_MS = 3000     # give up waiting for the arm to stop moving

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
    stall = StopWatch()
    best = None
    while not (aborted or watchdog.time() > WATCHDOG_MS):
        if check_abort():
            break
        err = target - dist_now()
        if abs(err) <= DRIVE_TOL or timer.time() > STEP_TIMEOUT_MS:
            break
        if best is None or abs(err) < best - STALL_PROGRESS_DRIVE:
            best = abs(err)
            stall.reset()
        elif stall.time() > STALL_MS:
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
    stall = StopWatch()
    best = None
    while not (aborted or watchdog.time() > WATCHDOG_MS):
        if check_abort():
            break
        err = hub.imu.heading() - target_heading
        if abs(err) <= TURN_TOL or timer.time() > STEP_TIMEOUT_MS:
            break
        if best is None or abs(err) < best - STALL_PROGRESS_TURN:
            best = abs(err)
            stall.reset()
        elif stall.time() > STALL_MS:
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


def relax_arm(which):
    """Coast the named arm motor ("right", "left", or "both") so it is
    completely free (a raised arm falls under its own weight), wait until
    it has stopped moving, then make the resting angle the new zero for
    later arm steps. Prints RELAX <angle(s)> each tick for debugging."""
    global right_arm_zero, left_arm_zero
    motors = []
    if which in ("right", "both"):
        motors.append(right_arm_motor)
    if which in ("left", "both"):
        motors.append(left_arm_motor)

    n = len(motors)
    for m in motors:
        m.stop()
    last = [m.angle() for m in motors]
    still = [0] * n
    timer = StopWatch()
    while timer.time() < RELAX_TIMEOUT_MS:
        if check_abort():
            break
        wait(RELAX_TICK_MS)
        for i in range(n):
            now = motors[i].angle()
            if abs(now - last[i]) > RELAX_SETTLE_TOL:
                still[i] = 0
                last[i] = now
            else:
                still[i] += RELAX_TICK_MS
        print("RELAX", which, timer.time(), last, still)
        if min(still) >= RELAX_QUIET_MS:
            break

    if which in ("right", "both"):
        right_arm_zero = right_arm_motor.angle()
    if which in ("left", "both"):
        left_arm_zero = left_arm_motor.angle()


def probe(which, ms):
    """Debug: print angle/speed/load every 100ms. First half holds the
    motor at its current angle (push it by hand and watch load), second
    half coasts."""
    m = right_arm_motor if which == "right" else left_arm_motor
    timer = StopWatch()
    m.hold()
    phase = "HOLD"
    while timer.time() < ms:
        if check_abort():
            break
        if phase == "HOLD" and timer.time() >= ms // 2:
            m.stop()
            phase = "COAST"
        print("PROBE", phase, timer.time(), m.angle(), m.speed(), m.load())
        wait(100)
    m.stop()


# PROGRAM: edit freely. Run in order, top to bottom.
#   ("relax_arm", "right"|"left"|"both")  -- coast that arm motor and
#     make wherever it ends up the new zero for later arm steps
PROGRAM = [
    ('probe', 'right', 12000),
]

for step in PROGRAM:
    if aborted or watchdog.time() > WATCHDOG_MS:
        break
    kind = step[0]
    if kind == "drive":
        drive(step[1], step[2])
    elif kind == "turn":
        turn(step[1], step[2])
    elif kind == "relax_arm":
        relax_arm(step[1])
    elif kind == "probe":
        probe(step[1], step[2])
    else:
        arm_to(step[1], step[2], step[3])

print("DETERMINISTIC_ABORTED" if aborted else "DETERMINISTIC_DONE")
