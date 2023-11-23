import simplekml
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_dir, "../.."))

from util import gcj02_to_wgs84

def SaveTrajecoryAsKml(traj_points, file_path):
    kml = simplekml.Kml()

    # add line elements
    line = kml.newlinestring(name="traj")
    for point in traj_points:
        lontitute, latitute = gcj02_to_wgs84(point[1], point[2])
        line.coords.addcoordinates([(lontitute, latitute, 0)])

    line.style.linestyle.color = simplekml.Color.red
    line.style.linestyle.width = 3
    kml.save(file_path)

def ParseTrajectories(traj_path):
    # 打开文件
    file = open(traj_path, "r")
    trajectrory_by_order_id = {}
    for line in file:
        result = line.split(",")
        user_id = result[0]
        order_id = result[1]
        time_stamp = int(result[2])
        longitude = float(result[3])
        latitude = float(result[4])
        if order_id not in trajectrory_by_order_id:
            trajectrory_by_order_id[order_id] = [(time_stamp, longitude, latitude)]
        else:
            trajectrory_by_order_id[order_id].append((time_stamp, longitude, latitude))
    print("len(trajectrory_by_order_id) = {}".format(len(trajectrory_by_order_id)))

    valid_traj_num = 0
    for order_id, traj_points in trajectrory_by_order_id.items():
        # 这里只保留轨迹点数大于1000的轨迹，实际上应该过滤，但是不过滤轨迹量太多了
        if len(traj_points) < 1000:
            continue
        traj_points.sort(key=lambda x: x[0])
        valid_traj_num += 1
        SaveTrajecoryAsKml(traj_points, "./data/trajectories/{}.kml".format(order_id))
    print("valid_traj_num = {}".format(valid_traj_num))

if __name__ == "__main__":
    traj_file = "./data/trajectories/gps_20161001.txt"
    ParseTrajectories(traj_file)