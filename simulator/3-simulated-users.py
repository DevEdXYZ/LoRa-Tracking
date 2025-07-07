# simulated_gps_feed_multi_async.py
#
# Emits "Data:" lines for three tracks at random times.
# Format:
# Data: <unit_id> <person_id> <sos> <lat> <lon> <elev> <heading> <steps>

import time, random, math

# ─── timing ---------------------------------------------------
MIN_SEND_S = 0.20          # shortest gap between packets
MAX_SEND_S = 3.00          # longest  gap between packets
METRES_PER_DEG_LAT = 111_111

DIST_M_RANGE     = (0.3, 1.5)   # metres per movement
STEP_RANGE       = (1, 3)       # steps per packet
ELEV_DRIFT_RANGE = (-0.2, 0.3)

# ─── initial “people” state ----------------------------------
people = [
    dict(unit=1, person=1, sos=0, lat=51.06864313, lon=-1.79400968,
         elev=1.0, heading=0.0, steps=0, next_emit=0.0),
    dict(unit=1, person=2, sos=0, lat=51.06858213, lon=-1.79406068,
         elev=1.0, heading=0.0, steps=0, next_emit=0.0),
    dict(unit=2, person=1, sos=0, lat=51.06873213, lon=-1.79392868,
         elev=1.0, heading=0.0, steps=0, next_emit=0.0),
]

# ─── helpers --------------------------------------------------
def advance(p):
    """Move one step and update stats."""
    dist_m = random.uniform(*DIST_M_RANGE)
    p["heading"] = random.uniform(0, 360)

    rad   = math.radians(p["heading"])
    d_lat = (dist_m * math.cos(rad)) / METRES_PER_DEG_LAT
    d_lon = (dist_m * math.sin(rad)) / (
        METRES_PER_DEG_LAT * math.cos(math.radians(p["lat"]))
    )

    p["lat"]  += d_lat
    p["lon"]  += d_lon
    p["elev"] += random.uniform(*ELEV_DRIFT_RANGE)
    p["steps"] += random.randint(*STEP_RANGE)

def schedule_next(p, now):
    p["next_emit"] = now + random.uniform(MIN_SEND_S, MAX_SEND_S)

# ─── main loop -----------------------------------------------
now = time.time()
for p in people:
    schedule_next(p, now)

while True:
    now = time.time()

    # Emit packets whose time has come
    for p in people:
        if p["next_emit"] <= now:
            print(
                "Data: {unit} {person:02d} {sos} {lat:.6f} {lon:.6f} "
                "{elev:.1f} {heading:.1f} {steps}".format(**p)
            )
            advance(p)
            schedule_next(p, now)

    # Sleep until the soonest next packet, capped to ~50 ms
    next_due = min(p["next_emit"] for p in people)
    sleep_time = next_due - time.time()
    if sleep_time < 0.05:
        sleep_time = 0.05
    time.sleep(sleep_time)
