import simplekml
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_dir, "../.."))

from util import gcj02_to_wgs84

class TrajectoryPoint:
    def __init__(self, longitude, latitude, altitude, time_stamp):
        self.longitude = longitude
        self.latitude = latitude
        self.altitude = altitude
        self.time_stamp = time_stamp

def SaveTrajecoryAsKml(traj_points, file_path):
    kml = simplekml.Kml()

    for point in traj_points:
        lontitute, latitute = gcj02_to_wgs84(point.longitude, point.latitude)
        new_point = kml.newpoint(coords=[(lontitute, latitute)])
        new_point.timestamp.when = point.time_stamp
    kml.save(file_path)

    # # add line elements
    # placemark = kml.newlinestring(name="traj")
    # for point in traj_points:
    #     lontitute, latitute = gcj02_to_wgs84(point.longitude, point.latitude)
    #     placemark.coords.addcoordinates([(lontitute, latitute, 0)])
    #     timespan = placemark.newtimespan()
    #     timespan.begin = point.time_stamp
    #     timespan.end = point.time_stamp
    #     # placemark.timespan.append({'begin': point.time_stamp, 'end': point.time_stamp}) 

    # placemark.style.linestyle.color = simplekml.Color.red
    # placemark.style.linestyle.width = 3
    # kml.save(file_path)

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
        altitude = 0
        traj_point = TrajectoryPoint(longitude, latitude, altitude, time_stamp)
        if order_id not in trajectrory_by_order_id:
            trajectrory_by_order_id[order_id] = [traj_point]
        else:
            trajectrory_by_order_id[order_id].append(traj_point)
    print("len(trajectrory_by_order_id) = {}".format(len(trajectrory_by_order_id)))

    valid_traj_num = 0
    for order_id, traj_points in trajectrory_by_order_id.items():
        # 这里只保留轨迹点数大于1000的轨迹，实际上不应该过滤，但是不过滤轨迹量太多了
        if len(traj_points) < 1000:
            continue
        traj_points.sort(key=lambda x: x.time_stamp)
        valid_traj_num += 1
        SaveTrajecoryAsKml(traj_points, "./data/trajectories/{}.kml".format(order_id))
    print("valid_traj_num = {}".format(valid_traj_num))

if __name__ == "__main__":
    traj_file = "./data/gps_20161001.txt"
    ParseTrajectories(traj_file)