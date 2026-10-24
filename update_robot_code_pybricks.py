from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Button, Color
from pybricks.tools import wait

hub = PrimeHub()
hub.system.set_stop_button(None)  # let us read the buttons ourselves

left_motor = Motor(Port.F)
right_motor = Motor(Port.E)
right_arm_motor = Motor(Port.C)
left_arm_motor = Motor(Port.D)


def wait_for_release(button):
    while button in hub.buttons.pressed():
        wait(10)


# Motorized "power assist" while recording: fights the gear train's own
# friction (not an electrical hold -- .stop() already coasts, see below)
# by nudging a motor with a bit of raw power in whatever direction it's
# currently being pushed, sampled every ASSIST_TICK_MS. Kept deliberately
# small and with a deadband so it (a) never starts moving a motor on its
# own from sensor noise, and (b) zeroes out the instant real motion stops
# instead of coasting onward under its own power. Tune ASSIST_DUTY if it
# still feels sticky (raise it) or if it ever seems to keep going briefly
# after you let go (lower it).
ASSIST_DUTY = 15        # % motor power nudge in the push direction
ASSIST_DEADBAND = 1     # encoder-deg/tick below which we call it stationary
ASSIST_TICK_MS = 20      # how often assist re-evaluates direction
ROW_EVERY = 16           # log a CSV row every N assist ticks (~320ms)


def assist_tick(motor, cur, prev):
    d = cur - prev
    if d > ASSIST_DEADBAND:
        motor.dc(ASSIST_DUTY)
    elif d < -ASSIST_DEADBAND:
        motor.dc(-ASSIST_DUTY)
    else:
        motor.dc(0)


# Browse counter. 0 = home ("H"). LEFT counts up, RIGHT counts back down
# towards home. RIGHT at home starts a new recording. CENTER replays the
# selected run (the computer generates and pushes the replay program in
# response to REPLAY_REQUEST, since the recorded data lives on the computer).
selected = 0

while True:
    hub.light.off()
    if selected == 0:
        hub.display.char("H")
    else:
        hub.display.number(selected)

    pressed = hub.buttons.pressed()

    if Button.RIGHT in pressed:
        wait_for_release(Button.RIGHT)
        wait(50)  # let switch bounce settle before the next poll

        if selected > 0:
            selected -= 1
            print("SELECTED: {}".format(selected))
            continue

        hub.light.on(Color.RED)
        hub.display.char("R")

        # A motor left in hold() from a previous replay (e.g. arm_to()
        # ends with .hold()) keeps actively resisting movement at the
        # hardware level even after that program exits -- explicitly
        # coast all four motors so they're always free to push by hand
        # for this new recording.
        left_motor.stop()
        right_motor.stop()
        right_arm_motor.stop()
        left_arm_motor.stop()

        print("RUN_START")

        prev_left = left_motor.angle()
        prev_right = right_motor.angle()
        prev_right_arm = right_arm_motor.angle()
        prev_left_arm = left_arm_motor.angle()
        assist_prev_left = prev_left
        assist_prev_right = prev_right
        assist_prev_right_arm = prev_right_arm
        assist_prev_left_arm = prev_left_arm
        hub.imu.reset_heading(0)
        row_tick = 0

        while Button.CENTER not in hub.buttons.pressed():
            left = left_motor.angle()
            right = right_motor.angle()
            right_arm = right_arm_motor.angle()
            left_arm = left_arm_motor.angle()

            assist_tick(left_motor, left, assist_prev_left)
            assist_tick(right_motor, right, assist_prev_right)
            assist_tick(right_arm_motor, right_arm, assist_prev_right_arm)
            assist_tick(left_arm_motor, left_arm, assist_prev_left_arm)
            assist_prev_left = left
            assist_prev_right = right
            assist_prev_right_arm = right_arm
            assist_prev_left_arm = left_arm

            row_tick += 1
            if row_tick >= ROW_EVERY:
                row_tick = 0
                heading = hub.imu.heading()

                dleft = left - prev_left
                dright = right - prev_right
                dright_arm = right_arm - prev_right_arm
                dleft_arm = left_arm - prev_left_arm

                prev_left = left
                prev_right = right
                prev_right_arm = right_arm
                prev_left_arm = left_arm

                print("ROW: Left: {}, Right: {}, DLeft: {}, DRight: {}, Heading: {}, "
                      "RightArm: {}, LeftArm: {}, DRightArm: {}, DLeftArm: {}".format(
                    left, right, dleft, dright, heading,
                    right_arm, left_arm, dright_arm, dleft_arm))

            wait(ASSIST_TICK_MS)

        left_motor.stop()
        right_motor.stop()
        right_arm_motor.stop()
        left_arm_motor.stop()

        print("RUN_STOP")
        wait_for_release(Button.CENTER)

    elif Button.LEFT in pressed:
        wait_for_release(Button.LEFT)
        wait(50)
        selected += 1
        print("SELECTED: {}".format(selected))

    elif Button.CENTER in pressed:
        wait_for_release(Button.CENTER)
        if selected > 0:
            hub.light.on(Color.BLUE)
            print("REPLAY_REQUEST: {}".format(selected))
            # The computer stops this program momentarily to push and run
            # the replay program, then restarts this one.

    else:
        wait(10)
