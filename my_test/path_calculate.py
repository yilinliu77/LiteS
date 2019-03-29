import os
import numpy as np
import math

with open("camera.log","r") as f:
    line=f.readline()
    out_string=""
    while len(line)>5:
        items = line.split(",")
        pos_x=float(items[1])*100
        pos_y=-float(items[2])*100
        pos_z = float(items[3])*100
        x = float(items[4])
        y = float(items[5])
        z = float(items[6])
        direction = np.asarray([float(x), float(y), float(z)])
        direction_in_xy = np.asarray([float(x), float(y), 0])

        pitch = math.acos(np.dot(direction / np.linalg.norm(direction), direction_in_xy / np.linalg.norm(direction_in_xy)))
        if y == 0:
            y=0.00001
        yaw = math.atan(x / y)
        
        pitch=pitch/math.pi*180
        yaw = yaw / math.pi * 180

        if direction[1] < 0:
            yaw+=180
        
        new_items = [items[0], str(pos_x), str(pos_y), str(pos_z), str(pitch), str(0), str(yaw)]
        out_string += ",".join(new_items) + "\n"
        line=f.readline()
    with open("camera_after_transaction.log","w") as fout:
        fout.write(out_string)
