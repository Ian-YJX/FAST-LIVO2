#!/usr/bin/env python
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped, PointStamped
import tf2_ros
import tf2_geometry_msgs
import numpy as np

# 全局累积点列表
all_rgb_points = []
all_r_points = []
all_g_points = []
all_b_points = []

# TF Buffer 和 Listener
tf_buffer = None
tf_listener = None

# 点云字段
rgb_fields = [
    PointField('x', 0, PointField.FLOAT32, 1),
    PointField('y', 4, PointField.FLOAT32, 1),
    PointField('z', 8, PointField.FLOAT32, 1),
    PointField('rgb', 12, PointField.UINT32, 1),
]

intensity_fields = [
    PointField('x', 0, PointField.FLOAT32, 1),
    PointField('y', 4, PointField.FLOAT32, 1),
    PointField('z', 8, PointField.FLOAT32, 1),
    PointField('intensity', 12, PointField.FLOAT32, 1)
]
def mag_callback(msg):
    global all_rgb_points, tf_buffer

    try:
        # 查找从 livox_frame -> camera_init 的变换
        trans = tf_buffer.lookup_transform("camera_init", msg.header.frame_id, rospy.Time(0), rospy.Duration(0.1))
    except Exception as e:
        # rospy.logwarn(f"TF lookup failed: {e}")
        return

    rgb_points = []
    r_points = []
    g_points = []
    b_points = []
    for p in pc2.read_points(msg, field_names=("x","y","z","Bx","By","Bz"), skip_nans=True):
        x, y, z, Bx, By, Bz = p

        # 构造一个 PointStamped
        ps = PointStamped()
        ps.header = msg.header
        ps.point.x, ps.point.y, ps.point.z = x, y, z

        # 坐标系变换
        pt_out = tf2_geometry_msgs.do_transform_point(ps, trans)

        # 计算磁场大小 -> 映射颜色
        # mag = math.sqrt(Bx**2 + By**2 + Bz**2)
        # val = min(mag / 40.0, 1.0)
        r = 255 * (Bx-25) / 15.0
        g = 255 * (By-25) / 15.0
        b = 255 * (Bz-25) / 15.0
        rgb = np.uint32((int(r) << 16) | (int(g) << 8) | int(b))
        rgb_points.append((pt_out.point.x, pt_out.point.y, pt_out.point.z, rgb))
        r_points.append((pt_out.point.x, pt_out.point.y, pt_out.point.z,r))
        g_points.append((pt_out.point.x, pt_out.point.y, pt_out.point.z,g))
        b_points.append((pt_out.point.x, pt_out.point.y, pt_out.point.z,b))

    # 累积所有点
    all_rgb_points.extend(rgb_points)
    all_r_points.extend(r_points)
    all_g_points.extend(g_points)
    all_b_points.extend(b_points)

    if all_rgb_points:
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "camera_init"   # 转换到全局
        cloud = pc2.create_cloud(header, rgb_fields, all_rgb_points)
        pub_0.publish(cloud)
    if all_r_points:
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "camera_init"   # 转换到全局
        cloud = pc2.create_cloud(header, intensity_fields, all_r_points)
        pub_1.publish(cloud)
    if all_g_points:
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "camera_init"   # 转换到全局
        cloud = pc2.create_cloud(header, intensity_fields, all_g_points)
        pub_2.publish(cloud)
    if all_b_points:
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "camera_init"   # 转换到全局
        cloud = pc2.create_cloud(header, intensity_fields, all_b_points)
        pub_3.publish(cloud)


if __name__ == "__main__":
    rospy.init_node("mag_field_visualizer")

    # 发布者
    pub_0 = rospy.Publisher("/mag_cloud/magnitude", PointCloud2, queue_size=1)
    pub_1 = rospy.Publisher("/mag_cloud/magnitude_x", PointCloud2, queue_size=1)
    pub_2 = rospy.Publisher("/mag_cloud/magnitude_y", PointCloud2, queue_size=1)
    pub_3 = rospy.Publisher("/mag_cloud/magnitude_z", PointCloud2, queue_size=1)

    # TF Buffer & Listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # 发布静态TF：aft_mapped 和 livox_frame 重合
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_tf = TransformStamped()
    static_tf.header.stamp = rospy.Time.now()
    static_tf.header.frame_id = "aft_mapped"
    static_tf.child_frame_id = "livox_frame"
    static_tf.transform.translation.x = 0.0
    static_tf.transform.translation.y = 0.0
    static_tf.transform.translation.z = 0.0
    static_tf.transform.rotation.x = 0.0
    static_tf.transform.rotation.y = 0.0
    static_tf.transform.rotation.z = 0.0
    static_tf.transform.rotation.w = 1.0
    static_broadcaster.sendTransform(static_tf)

    # 订阅磁场点云
    rospy.Subscriber("/mag_array/MagPoints_pc2", PointCloud2, mag_callback)

    rospy.spin()
