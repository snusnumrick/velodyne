'''
Example of how to use the velodyne library to visualize a point cloud.

Read velodyne packets from a rosbag file and visualize them using Open3D.
'''

import numpy as np

from velodyne_scan import VelodyneCloudConverter
import rosbag
import open3d as o3d
import time

sample_bag_fn = 'examples/sample.bag'
topic_name = '/velodyne_packets'
max_distance = 30  # ignore points further than 30 meters

escape = False


def escape_callback(_):
    # exit on Esc key press
    global escape
    escape = True
    return False  # Returning False to indicate no need for further processing


# prepare the converter
converter = VelodyneCloudConverter()

# prepare the visualizer
vis = o3d.visualization.VisualizerWithKeyCallback()
vis.create_window()
vis_options = vis.get_render_option()
vis_options.point_size = 1.0
vis_options.background_color = np.array([0, 0, 0])
vis.register_key_callback(256, escape_callback)

# load the first point cloud
with rosbag.Bag(sample_bag_fn) as sample_bag:
    for _, velodyne_message, velodyne_time in sample_bag.read_messages(topic_name):
        break

# convert the point cloud to Open3D format
cloud = converter.decode(velodyne_message)
o3d_cloud = cloud.to_open3d(filter_func=lambda p: p.distance < max_distance)
vis.add_geometry(o3d_cloud)

# main loop
while not escape:
    vis.poll_events()
    vis.update_renderer()
    time.sleep(0.01)

# clean up
vis.destroy_window()
