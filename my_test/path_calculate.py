import os
import numpy as np
import math

with open("camera.log","r") as f:
    line=f.readline()
    out_string=""
    while len(line)>5:
        items = line.split(",")
        pos_x = float(items[1])*100
        pos_y = -float(items[2])*100
        pos_z = float(items[3])*100
        x = float(items[4])
        y = -float(items[5])
        z = float(items[6])
        direction = np.asarray([float(x), float(y), float(z)])
        direction_in_xy = np.asarray([float(x), float(y), 0])

        # print(items[0])
        if (z == -1):
            pitch = 90
        else:
            pitch = math.acos(min(np.dot(direction / np.linalg.norm(direction)
                , direction_in_xy / np.linalg.norm(direction_in_xy)),1.0))
        if y == 0:
            yaw=0
        else:
            yaw = math.atan(y / x)
        
        pitch=pitch/math.pi*180
        yaw = yaw / math.pi * 180

        if direction[0] < 0:
            yaw+=180
        
        new_items = [items[0], str(pos_x), str(pos_y), str(pos_z), str(pitch), str(0), str(yaw)]
        out_string += ",".join(new_items) + "\n"
        line=f.readline()
    with open("camera_after_transaction.log","w") as fout:
        fout.write(out_string)
