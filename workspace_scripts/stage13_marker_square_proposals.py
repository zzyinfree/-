"""Find dark square candidates in selected down frames for visual review only."""
import cv2
import glob
import json
from pathlib import Path
import sys

root = Path(sys.argv[1])
out = Path(sys.argv[2])
specs = [('run_024_purple_stage13', 78, 96),
         ('run_025_purple_stage13', 69, 90)]
if len(sys.argv) > 3:
    specs = []
    for spec in sys.argv[3:]:
        run, start, stop = spec.split(':')
        specs.append((run, int(start), int(stop)))
rows = []
tiles = []
for run, start, stop in specs:
    for i in range(start, stop):
        path = root/'recordings'/run/'down'/f'frame_{i:04d}.png'
        if not path.exists():
            continue
        im = cv2.imread(str(path))
        gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        mask = (gray < 80).astype('uint8')*255
        contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if not 900 <= area <= 100000:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if not 0.6 <= w/max(h, 1) <= 1.65 or min(w, h) < 38:
                continue
            fill = area/(w*h)
            if fill < 0.32:
                continue
            # A marker has substantial nearly-black pixels inside the square.
            crop = gray[y:y+h, x:x+w]
            very_dark = (crop < 45).mean()
            score = area * min(fill, 0.9) * min(very_dark, 0.8)
            candidates.append((score, [x, y, x+w, y+h]))
        candidates.sort(reverse=True)
        best = candidates[0][1] if candidates else None
        row = {'run': run, 'image': str(path), 'frame': i,
               'bbox_candidate': best,
               'candidates': [b for _, b in candidates[:3]]}
        rows.append(row)
        preview = im.copy()
        if best:
            cv2.rectangle(preview, tuple(best[:2]), tuple(best[2:]), (0, 255, 0), 3)
        preview = cv2.resize(preview, (320, 240))
        cv2.putText(preview, f'{run[4:7]}:{i}', (5, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 255), 2)
        tiles.append(preview)
while len(tiles)%4:
    tiles.append(tiles[-1].copy())
cv2.imwrite(str(out)+'.jpg', cv2.vconcat([cv2.hconcat(tiles[j:j+4])
                                          for j in range(0, len(tiles), 4)]))
out.with_suffix('.json').write_text(json.dumps(rows, indent=2))
print(len(rows), sum(r['bbox_candidate'] is not None for r in rows))
