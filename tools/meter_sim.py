"""Python copy of the circuit-meter car type check (TWCCharger::meter_detect_ + circuit_window_).
Runs a few scenarios: 1-phase / 3-phase car, heat pump switching, slow meter. python meter_sim.py"""
import math, random

V = 240.0


def window(samples, frm, to, carry=True):
    vals, before = [], None
    for t, w in samples:
        if t < frm:
            before = w
            continue
        if t > to:
            break
        if carry and before is not None and not vals:
            vals.append(before)
        vals.append(w)
    if carry and not vals and before is not None:
        vals = [before, before]
    if len(vals) < 2:
        return None
    return sum(vals) / len(vals), max(vals) - min(vals)


def run(name, factor, meter_every, heatpump_at=None, i0=0.0, i1=10.0, seed=1):
    random.seed(seed)
    step = 300  # s: current goes i0 -> i1 (car ramps over ~10 s)
    samples, ring = [], []
    def cur(t):  # ramps from i0 to i1 over 10 s
        return i0 if t < step else i0 + (i1 - i0) * min(1.0, (t - step) / 10)
    for t in range(0, 700, meter_every):  # meter readings
        load = 1300 + random.uniform(-40, 40)
        if heatpump_at is not None and t >= heatpump_at:
            load += 1500
        samples.append((t, load + factor * V * cur(t)))
    pending, res = None, "no step"
    for t in range(0, 700, 5):  # heartbeat ring every 5 s
        ring.append(cur(t))
        if pending is None:
            if len(ring) >= 4 and abs(ring[-1] - ring[-4]) >= 3:
                pending = (t, t - 15, ring[-4])
            continue
        trig, t0, ib = pending
        if t - trig > 400:
            res = "timeout (current not steady / too few meter readings)"
            break
        if t - trig < 90:
            continue
        last = ring[-12:]
        if max(last) - min(last) > 1:
            continue
        ia = sum(last) / 12
        # before: 2 min up to the step (last older reading counts); after: from step + 30 s, real readings only
        b, a = window(samples, t0 - 120, t0), window(samples, trig + 30, t, carry=False)
        if b is None or a is None:
            continue  # wait for more meter readings
        r = (a[0] - b[0]) / (V * (ia - ib))
        if b[1] > 400 or a[1] > 400:
            res = f"r={r:.2f} meter not steady (spread {b[1]:.0f}/{a[1]:.0f}) -> skipped"
        elif 0.8 <= r <= 1.2:
            res = f"r={r:.2f} -> 1-phase"
        elif 1.39 <= r <= 2.08:
            res = f"r={r:.2f} -> 3-phase"
        else:
            res = f"r={r:.2f} -> no match, skipped"
        break
    print(f"{name:42} {res}")


run("1-phase car 0->10 A, meter 5 s", 1.0, 5)
run("3-phase car 0->10 A, meter 5 s", math.sqrt(3), 5)
run("3-phase car 6->10 A, meter 10 s", math.sqrt(3), 10, i0=6)
run("1-phase car 0->10 A, meter 60 s", 1.0, 60)
run("3-phase car 0->10 A, meter 60 s", math.sqrt(3), 60)
run("1-phase car, heat pump on in after window", 1.0, 5, heatpump_at=380)
run("1-phase car, heat pump on just at step", 1.0, 5, heatpump_at=300)
run("3-phase car 10->6 A (down), meter 5 s", math.sqrt(3), 5, i0=10, i1=6)
