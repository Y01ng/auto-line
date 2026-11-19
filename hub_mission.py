# Runs ON the hub (official SPIKE 3 firmware). mission.html uploads it to slot 19 and starts it.
# The phone overwrites slot 18's file with "<seq> <op> <ints...>;"
#   V a b leg limit   stream raw motor speeds (deg/s) for ports A, B. `leg` = id of this run;
#                     limit > 0: stop by itself after that many wheel degrees (prints "L<leg> <deg>").
#                     While moving it prints odometry "O<leg> <deg>" every ~120 ms.
#   M mm speed        straight move by wheel rotation (mm, - = backwards)      -> "D<seq> 1"
#   T deg speed       turn in place by wheel rotation (deg, + = right)         -> "D<seq> 1"
#   G dir power hold maxdeg flag   gripper at limited force (power = duty %) until it stops moving,
#                             maxdeg is reached or 3 s pass. flag 1 = opening (a stall marks "fully
#                             open"), 2 = closing (maxdeg counted from fully open).
#                             prints "GR <deg> <why> <deg from fully open>"       -> "D<seq> 1"
#   S                 stop everything                                          -> "D<seq> 1"
#   Q                 quit
# Any new command aborts a running M / T / G ("D<seq> 0").
# Safety: V speeds stop if no fresh command for DEADMAN_MS (phone gone / frozen / screen off).
from hub import light_matrix, port, sound
import device
import motor
import runloop
import time

CMD_FILE = "/flash/program/18/program.py"
# >>> CONFIG: the phone page rewrites the next line before uploading >>>
CFG = {"swap": 0, "inv": 0, "grip": "auto", "wheel": 56, "track": 128}
DEADMAN_MS = 700
MAX_SPEED = 1000
MOTOR_IDS = (48, 49, 65, 75, 76)
PORTS = {"A": port.A, "B": port.B, "C": port.C, "D": port.D, "E": port.E, "F": port.F}
A, B = port.A, port.B
A_FWD = -1 if CFG["inv"] else 1      # motor A sign for "car forward" (B is mirrored)
B_FWD = -A_FWD
DEG_PER_MM = 360 / (3.14159265 * CFG["wheel"])
ST = {"seq": None}


def has_motor(p):
    try:
        return device.id(p) in MOTOR_IDS
    except Exception:
        return False


def sgn(x):
    return (x > 0) - (x < 0)


def pos(p):
    return motor.relative_position(p)


def read_cmd():
    try:
        with open(CMD_FILE) as f:
            s = f.read()
    except OSError:
        return None
    if not s.endswith(";"):     # empty / half-written file
        return None
    p = s[:-1].split()
    try:
        return int(p[0]), p[1], [int(x) for x in p[2:]]
    except (ValueError, IndexError):
        return None


def new_cmd():
    c = read_cmd()
    return c is not None and c[0] != ST["seq"]


def set_speeds(a, b):
    for p, v in ((A, a), (B, b)):
        if v:
            motor.run(p, max(-MAX_SPEED, min(MAX_SPEED, v)))
        else:
            motor.stop(p)


def run(p, v):
    motor.run(p, int(max(-MAX_SPEED, min(MAX_SPEED, v))), acceleration=3000)


async def wheels(sa, sb, target, speed):
    """Turn both wheels `target` degrees (signs sa / sb), kept equal, slowing down at the end."""
    a0, b0 = pos(A), pos(B)
    t0 = time.ticks_ms()
    limit_ms = 1500 + int(target * 2500 / max(60, speed))
    ok = True
    while True:
        da, db = abs(pos(A) - a0), abs(pos(B) - b0)
        rem = target - (da + db) / 2
        if rem <= 2:
            break
        if time.ticks_diff(time.ticks_ms(), t0) > limit_ms or new_cmd():
            ok = False
            break
        v = min(speed, 70 + rem * 4)
        corr = max(-50, min(50, (da - db) * 3))
        run(A, sa * (v - corr))
        run(B, sb * (v + corr))
        await runloop.sleep_ms(10)
    motor.stop(A)
    motor.stop(B)
    await runloop.sleep_ms(120)   # settle
    return ok


GZ = {"open": None}     # gripper position at its fully-open end stop (set by an opening move that stalls)
LEAD_LOOPS = 1          # stop this many 10 ms steps of travel before the angle limit (coasting)


async def do_grip(g, direction, power, hold, maxdeg, flag):
    """Move the gripper with LIMITED force: power = motor duty in % (it cannot push harder than
    that), until it stops moving (object / end stop), maxdeg is reached, or 3 s pass.
    flag 1 = opening move (a stall marks the fully-open position), flag 2 = closing move
    (maxdeg is counted from that fully-open position), 0 = plain relative move.
    Prints "GR <degrees moved> <stall|angle|time|cmd> <degrees from fully open or ?>"."""
    if g is None:
        return False
    start = p0 = last = pos(g)
    target = maxdeg
    if flag == 2 and maxdeg > 0 and GZ["open"] is not None:
        target = maxdeg - abs(start - GZ["open"])    # limit counted from the fully-open end stop
        if target <= 2:
            print("GR 0 angle " + str(abs(start - GZ["open"])))
            return True
    t0 = moved = time.ticks_ms()
    try:
        motor.set_duty_cycle(g, sgn(direction) * max(0, min(100, power)) * 100)
    except Exception:
        motor.run(g, sgn(direction) * 250)           # firmware without duty control: speed mode
    why = "time"
    n = 0
    while True:
        await runloop.sleep_ms(10)
        now = time.ticks_ms()
        p = pos(g)
        lead = abs(p - last) * LEAD_LOOPS            # it coasts on after power is cut: stop early
        last = p
        if abs(p - p0) > 2:
            p0, moved = p, now
        elif time.ticks_diff(now, moved) > 300:      # stopped moving: touching the object / end stop
            why = "stall"
            break
        if target > 0 and abs(p - start) + lead >= target:
            why = "angle"
            break
        if time.ticks_diff(now, t0) > 3000:
            break
        n += 1
        if n % 3 == 0 and new_cmd():
            why = "cmd"
            break
    if hold:
        try:
            motor.stop(g, stop=motor.HOLD)           # stay at this position (no extra squeeze)
        except Exception:
            motor.stop(g)
    else:
        motor.stop(g)
    await runloop.sleep_ms(150)                      # let it settle before measuring
    if flag == 1 and why == "stall":
        GZ["open"] = pos(g)                          # fully open: this is angle 0 from now on
    rel = "?" if GZ["open"] is None else str(abs(pos(g) - GZ["open"]))
    print("GR " + str(pos(g) - start) + " " + why + " " + rel)
    return why != "cmd"


async def main():
    if not (has_motor(A) and has_motor(B)):
        light_matrix.show_image(light_matrix.IMAGE_NO)
        print("NO_WHEELS")
        return
    gname = CFG["grip"]          # "auto" = first motor found on C..F, "" = none, or a port letter
    grip = None
    if gname == "auto":
        for n in ("C", "D", "E", "F"):
            if has_motor(PORTS[n]):
                gname, grip = n, PORTS[n]
                break
    else:
        grip = PORTS.get(gname)
        if grip is not None and not has_motor(grip):
            grip = None
    light_matrix.show_image(light_matrix.IMAGE_HAPPY)
    await sound.beep(660, 80, 50)
    await sound.beep(880, 80, 50)
    print("READY " + ("GRIP " + gname if grip is not None else "NOGRIP"))

    vcmd = (0, 0)
    applied = (0, 0)
    fresh = time.ticks_ms()
    leg, la0, lb0, limit, leg_done = -1, 0, 0, 0, False
    last_odo = fresh
    while True:
        now = time.ticks_ms()
        c = read_cmd()
        if c and c[0] != ST["seq"]:
            seq, op, a = c
            ST["seq"] = seq
            fresh = now
            print("A" + str(seq))
            if op == "Q":
                break
            if op == "V" and len(a) >= 4:
                if a[2] != leg:
                    leg, la0, lb0, leg_done = a[2], pos(A), pos(B), False
                limit = a[3]
                vcmd = (a[0], a[1])
            else:
                vcmd = (0, 0)
                if applied != (0, 0):
                    set_speeds(0, 0)
                    applied = (0, 0)
                ok = True
                try:
                    if op == "M":
                        d = sgn(a[0])
                        ok = await wheels(A_FWD * d, B_FWD * d, abs(a[0]) * DEG_PER_MM, a[1])
                    elif op == "T":
                        d = sgn(a[0])                       # right turn: left wheel forward
                        wa, wb = (d, -d) if CFG["swap"] else (-d, d)
                        ok = await wheels(A_FWD * wa, B_FWD * wb, abs(a[0]) * CFG["track"] / CFG["wheel"], a[1])
                    elif op == "G":
                        ok = await do_grip(grip, a[0], a[1], a[2], a[3], a[4] if len(a) > 4 else 0)
                    elif op == "S":
                        if grip is not None:
                            motor.stop(grip)
                except Exception as e:
                    set_speeds(0, 0)
                    ok = False
                    print("ERR " + str(e))
                print("D" + str(seq) + (" 1" if ok else " 0"))
                fresh = time.ticks_ms()
                continue

        eff = (0, 0) if leg_done else vcmd
        if eff != (0, 0) and time.ticks_diff(now, fresh) > DEADMAN_MS:
            eff = (0, 0)
            if applied != eff:
                print("DEADMAN")
        try:
            if eff != applied:
                set_speeds(eff[0], eff[1])
                applied = eff
            if applied != (0, 0):
                travel = (abs(pos(A) - la0) + abs(pos(B) - lb0)) // 2
                if limit > 0 and travel >= limit:
                    set_speeds(0, 0)
                    applied = (0, 0)
                    leg_done = True
                    print("L" + str(leg) + " " + str(travel))
                elif time.ticks_diff(now, last_odo) >= 120:
                    last_odo = now
                    print("O" + str(leg) + " " + str(travel))
        except Exception as e:          # e.g. a motor cable came loose: report, don't crash
            vcmd = (0, 0)
            applied = (0, 0)
            if time.ticks_diff(now, last_odo) >= 1000:
                last_odo = now
                print("ERR " + str(e))
        await runloop.sleep_ms(10)

    set_speeds(0, 0)
    if grip is not None:
        motor.stop(grip)
    light_matrix.show_image(light_matrix.IMAGE_ASLEEP)
    print("BYE")


runloop.run(main())
