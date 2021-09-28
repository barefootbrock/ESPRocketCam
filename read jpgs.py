import numpy as np
import cv2

file = "data/video007.029"
f = open("C:/Users/David/OneDrive - Dallas College/Documents/Projects/Water Rocket/" + file + ".jpgs", "rb")
i = 0

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter("C:/Users/David/OneDrive - Dallas College/Documents/Projects/Water Rocket/" + file + ".avi", fourcc, 20.0, (640,  480))

while True:
    n = f.read(4)
    if len(n) == 0:
        break
    
    if len(n) != 4:
        print("Bad size")
        break
    size = (n[3]<<24) + (n[2]<<16) + (n[1]<<8) + n[0]

    data = f.read(size)
    if len(data) != size:
        print("Bad frame")
        break

    file_bytes = np.array(list(data), dtype=np.uint8)
    frame = cv2.imdecode(file_bytes, cv2.IMREAD_COLOR)
    if frame is None:
        print("Bad frame")
        continue
##    frame = cv2.resize(frame, (800, 600))
    out.write(frame)
    cv2.imshow("frame", frame)
    cv2.waitKey(1)

    i += 1

out.release()
f.close()
print("%i frames saved" % i)
