import json
from pathlib import Path

work = Path(__file__).resolve().parent / 'smoke'
result = {}
for robot in ('aerial05', 'aerial08'):
    rows = [json.loads(line) for line in
            (work / 'frontends' / robot / 'frontend/native_updates.jsonl').read_text().splitlines()]
    accepted = [i for i, row in enumerate(rows) if row['lidar_updated']]
    first, last = max(zip(accepted, accepted[1:]),
                      key=lambda pair: rows[pair[1]]['sensor_stamp_ns'] - rows[pair[0]]['sensor_stamp_ns'])
    intervening = rows[first + 1:last]
    origin = rows[0]['sensor_stamp_ns']
    result[robot] = {
        'successful_scan_ids': [rows[first]['scan_id'], rows[last]['scan_id']],
        'interval_since_initialization_s': [(rows[i]['sensor_stamp_ns'] - origin) / 1e9 for i in (first, last)],
        'gap_s': (rows[last]['sensor_stamp_ns'] - rows[first]['sensor_stamp_ns']) / 1e9,
        'intervening_processed_scans': len(intervening),
        'intervening_feature_counts': sorted(set(row['features'] for row in intervening)),
        'intervening_successful_updates': sum(row['lidar_updated'] for row in intervening),
        'intervening_max_processing_s': max((row['processing_s'] for row in intervening), default=None),
    }
(work / 'update-gap-details.json').write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps(result, indent=2))
