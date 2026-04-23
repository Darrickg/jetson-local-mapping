
from rosbags.highlevel import AnyReader
from pathlib import Path
from rosbags.typesys import Stores, get_typestore
from rosbags.rosbag2 import Writer
import sys
import time
def get_seconds(point):
    return point.header.stamp.sec + (point.header.stamp.nanosec /  10**9)

def main():
    threshold = 0.02
    bag_path = Path(f"{sys.argv[1]}")
    output_path = Path(f"{sys.argv[2]}-{round(time.time())}")
    lidar_points = []
    camera_points = []
    good_pairings = []
    conns = {}
    metadata = {}
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    with AnyReader([bag_path], default_typestore=typestore) as reader:
        for connection, timestamp, rawdata in reader.messages():
            msg = reader.deserialize(rawdata, connection.msgtype)
            if "zed" in msg.header.frame_id:
                camera_points.append(msg)
            if "rslidar" in msg.header.frame_id:
                lidar_points.append(msg)
            metadata[str(msg)] = (connection, timestamp, rawdata)
            
    for point1 in camera_points:
        for point2 in lidar_points:
            point1time = get_seconds(point1)
            point2time = get_seconds(point2)
            if ((point1time) <= (point2time + threshold)) and ((point1time) >= (point2time - threshold)) :
                good_pairings.append([point1, point2])

    with AnyReader([bag_path], default_typestore=typestore) as reader, Writer(output_path, version=2) as writer:
        for connection in reader.connections:
            conns[connection.id] = writer.add_connection(
                connection.topic,
                connection.msgtype,
                typestore=typestore
            )
        for pt1, pt2 in good_pairings:
            conn1, timestamp1, rawdata1 = metadata[str(pt1)]
            conn2, timestamp2, rawdata2 = metadata[str(pt2)]
            writer.write(conns[conn1.id], timestamp1, rawdata1)
            writer.write(conns[conn2.id], timestamp2, rawdata2)
    print(f"Good Pairs: {len(good_pairings)}")

if __name__ == '__main__':
    main()