import runloop
import motor
from hub import port
from hub import light_matrix

async def main():
    await light_matrix.write("bye")

    # Initialize previous positions directly with integers to avoid "None" type warnings
    prev_left = motor.relative_position(port.C)
    prev_right = motor.relative_position(port.E)
    
    # Run forever. Pressing the center button on the physical hub will 
    # natively stop the program, so we don't need to manually check for it!
    while True:
        left = motor.relative_position(port.C)
        right = motor.relative_position(port.E)

        # Compute deltas
        dleft = left - prev_left
        dright = right - prev_right
            
        # Update previous positions for the next loop
        prev_left = left
        prev_right = right

        print("UPDATE_SHEET: Left: {}, Right: {}, DLeft: {}, DRight: {}".format(left, right, dleft, dright))

        await runloop.sleep_ms(333)

runloop.run(main())
