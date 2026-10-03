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
        print("RUN_START")

        prev_left = left_motor.angle()
        prev_right = right_motor.angle()
        prev_right_arm = right_arm_motor.angle()
        prev_left_arm = left_arm_motor.angle()
        hub.imu.reset_heading(0)

        while Button.CENTER not in hub.buttons.pressed():
            left = left_motor.angle()
            right = right_motor.angle()
            right_arm = right_arm_motor.angle()
            left_arm = left_arm_motor.angle()
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

            wait(333)

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
