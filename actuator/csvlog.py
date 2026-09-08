"""Read the CSVs the test scripts write.

Every script logs one row per sample to data/<name>_<timestamp>.csv. Numbers
that could not be read come back as nan rather than being dropped, so a gap in
the telemetry stays visible in the plot instead of silently closing up.
"""
import csv

def load_csv(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    out = []
    for r in rows:
        d = {}
        for k, v in r.items():
            if k in ("block", "phase"):
                d[k] = v
            else:
                try:
                    d[k] = float(v)
                except (TypeError, ValueError):
                    d[k] = float("nan")
        out.append(d)
    return out
