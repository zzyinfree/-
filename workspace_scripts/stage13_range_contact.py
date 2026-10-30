import cv2
import glob
import os
import sys

directory, start, stop, step, output = sys.argv[1], *map(int, sys.argv[2:5]), sys.argv[5]
frames = sorted(glob.glob(os.path.join(directory, 'frame*.png')))
tiles = []
for i in range(start, min(stop, len(frames)), step):
    im = cv2.resize(cv2.imread(frames[i]), (320, 240))
    cv2.putText(im, str(i), (5, 25), cv2.FONT_HERSHEY_SIMPLEX,
                0.7, (0, 255, 255), 2)
    tiles.append(im)
while len(tiles) % 4:
    tiles.append(tiles[-1].copy())
cv2.imwrite(output, cv2.vconcat([cv2.hconcat(tiles[j:j+4])
                                   for j in range(0, len(tiles), 4)]))
