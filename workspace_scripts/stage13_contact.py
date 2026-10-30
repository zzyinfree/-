import cv2
import glob
import os
import sys

directory = sys.argv[1]
frames = sorted(glob.glob(os.path.join(directory, 'frame*.png')))
stride = max(1, len(frames) // 20)
images = []
for i in range(0, len(frames), stride):
    frame = cv2.resize(cv2.imread(frames[i]), (320, 240))
    cv2.putText(frame, str(i), (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                0.8, (0, 255, 255), 2)
    images.append(frame)
while len(images) % 4:
    images.append(images[-1].copy())
canvas = cv2.vconcat([cv2.hconcat(images[j:j + 4])
                      for j in range(0, len(images), 4)])
cv2.imwrite(os.path.join(directory, 'contact.jpg'), canvas)
