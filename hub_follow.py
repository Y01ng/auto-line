# Runs ON the hub (official SPIKE 3 firmware). The phone page uploads it to slot 19 and starts it.
# The phone overwrites slot 18's file with "<seq> <speedA> <speedB>;"  (raw motor speeds, deg/s).
#   seq < 0 = quit.
# Safety: no fresh command for DEADMAN_MS while moving -> stop (phone gone, frozen, screen off).
from hub import light_matrix, port, sound
import device
import motor
import runloop
import time

CMD_FILE = "/flash/program/18/program.py"
DEADMAN_MS = 700
MAX_SPEED = 1000
MOTOR_IDS = (48, 49, 65, 75, 76)


def has_motor(p):
    try:
        return device.id(p) in MOTOR_IDS
    except Exception:
        return False


def clamp(v):
    return max(-MAX_SPEED, min(MAX_SPEED, v))


def read_cmd():
    try:
        with open(CMD_FILE) as f:
            s = f.read()
    except OSError:
        return None
    if not s.endswith(";"):     # empty / half-written file
        return None
    try:
        seq, a, b = [int(x) for x in s[:-1].split()]
    except ValueError:
        return None
    return seq, a, b


def set_speeds(a, b):
    for p, v in ((port.A, a), (port.B, b)):
        if v:
            motor.run(p, clamp(v))
        else:
            motor.stop(p)


async def main():
    if not (has_motor(port.A) and has_motor(port.B)):
        light_matrix.show_image(light_matrix.IMAGE_NO)
        print("NO_WHEELS")
        return
    light_matrix.show_image(light_matrix.IMAGE_HAPPY)
    await sound.beep(660, 80, 50)
    await sound.beep(880, 80, 50)
    print("READY")

    last_seq = None
    cmd = (0, 0)
    applied = (0, 0)
    fresh = time.ticks_ms()
    while True:
        now = time.ticks_ms()
        c = read_cmd()
        if c and c[0] != last_seq:
            last_seq = c[0]
            fresh = now
            if last_seq < 0:
                break
            cmd = c[1:]
            print("A" + str(last_seq))

        eff = cmd
        if (eff[0] or eff[1]) and time.ticks_diff(now, fresh) > DEADMAN_MS:
            eff = (0, 0)
            if applied != eff:
                print("DEADMAN")

        if eff != applied:
            moving_before = applied != (0, 0)
            set_speeds(eff[0], eff[1])
            applied = eff
            if (applied != (0, 0)) != moving_before:
                light_matrix.show_image(light_matrix.IMAGE_GO_UP if applied != (0, 0)
                                        else light_matrix.IMAGE_SQUARE_SMALL)
        await runloop.sleep_ms(15)

    set_speeds(0, 0)
    light_matrix.show_image(light_matrix.IMAGE_ASLEEP)
    print("BYE")


runloop.run(main())
