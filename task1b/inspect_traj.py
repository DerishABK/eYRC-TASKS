import csv

rows = []
with open('trajectory.csv') as f:
    for r in csv.DictReader(f):
        rows.append({k: float(v) for k, v in r.items()})

times = [0, 10, 14, 15, 16, 17, 18, 20, 25, 30, 40, 59, 60, 61, 65, 70, 80, 94, 95, 96, 100, 110, 119]
print("     t         y   target_y      yaw    speed  steering")
for r in rows:
    if any(abs(r['t'] - t) < 0.06 for t in times):
        print("%6.1f  %8.4f  %8.4f  %8.4f  %7.4f  %9.4f" % (
            r['t'], r['y'], r['target_y'], r['yaw'], r['speed'], r['steering']))
