#!/usr/bin/env python

import rospy
import pcl
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs

class PointCloudFusion:
    def __init__(self):
        rospy.init_node('point_cloud_fusion', anonymous=True)

        self.pcd1 = None
        self.pcd2 = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Update these topic names to match your system
        rospy.Subscriber('/D435_camera_down/depth/color/points', PointCloud2, self.cloud1_callback)
        rospy.Subscriber('/D435_camera_top/depth/color/points', PointCloud2, self.cloud2_callback)

        # Update the output topic name if needed
        self.pub = rospy.Publisher('/fused_point_cloud', PointCloud2, queue_size=1)

    def cloud1_callback(self, msg):
        self.pcd1 = msg
        if self.pcd1 is not None:
            rospy.loginfo("Received message on /D435_camera_down/depth/color/points")

    def cloud2_callback(self, msg):
        self.pcd2 = msg
        if self.pcd2 is not None:
            rospy.loginfo("Received message on /D435_camera_top/depth/color/points")

    def convert_ros_to_pcl(self, ros_cloud):
        points_list = []

        for data in pc2.read_points(ros_cloud, skip_nans=True):
            points_list.append([data[0], data[1], data[2]])

        pcl_data = pcl.PointCloud()
        pcl_data.from_list(points_list)

        return pcl_data

    def convert_pcl_to_ros(self, pcl_cloud, frame_id):
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id

        ros_cloud = pc2.create_cloud_xyz32(header, pcl_cloud.to_list())
        return ros_cloud

    def fuse_clouds(self):
        if self.pcd1 is not None and self.pcd2 is not None:
            try:
                # Update the target frame to the correct one
                transform_1 = self.tf_buffer.lookup_transform('trunk',
                                                            self.pcd1.header.frame_id,
                                                            rospy.Time(0),
                                                            rospy.Duration(10.0))
                transform_2 = self.tf_buffer.lookup_transform('trunk',
                                                            self.pcd2.header.frame_id,
                                                            rospy.Time(0),
                                                            rospy.Duration(10.0))
                rospy.loginfo("Transforms successfully obtained")
                rospy.loginfo(f"pcd1 frame_id: {self.pcd1.header.frame_id}")
                rospy.loginfo(f"pcd2 frame_id: {self.pcd2.header.frame_id}")
                rospy.loginfo(f"Transform 1: {transform_1}")
                rospy.loginfo(f"Transform 2: {transform_2}")
                transformed_pcd1 = tf2_sensor_msgs.do_transform_cloud(self.pcd1, transform_1)
                transformed_pcd2 = tf2_sensor_msgs.do_transform_cloud(self.pcd2, transform_2)
                rospy.loginfo(f"Transformed point cloud 1 size: {len(transformed_pcd1.data)}")
                rospy.loginfo(f"Transformed point cloud 2 size: {len(transformed_pcd2.data)}")

                pcl1 = self.convert_ros_to_pcl(transformed_pcd1)
                pcl2 = self.convert_ros_to_pcl(transformed_pcd2)

                # Convert PCL point clouds to numpy arrays for concatenation
                pcl1_np = np.asarray(pcl1.to_array())
                pcl2_np = np.asarray(pcl2.to_array())

                combined_np = np.vstack((pcl1_np, pcl2_np))

                # Convert the combined numpy array back to PCL point cloud
                combined_pcl = pcl.PointCloud()
                combined_pcl.from_array(combined_np.astype(np.float32))

                # Update the frame_id to your desired output frame
                ros_cloud = self.convert_pcl_to_ros(combined_pcl, 'trunk')
                self.pub.publish(ros_cloud)
                rospy.loginfo("Published fused point cloud")
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn(f"Transform not available: {e}")

    def run(self):
        rate = rospy.Rate(10)  # 60 Hz
        while not rospy.is_shutdown():
            self.fuse_clouds()
            rate.sleep()

if __name__ == '__main__':
    fusion_node = PointCloudFusion()
    fusion_node.run()
