import rclpy
from geometry_msgs.msg import TransformStamped
import tf2_ros

class StaticTransformPublisher:
    def __init__(self) -> None:
        self.node = rclpy.create_node('static_transform_publisher_node')

        # Declare parameters with defaults (optional, but good for fallback)
        self.node.declare_parameter('header_frame_id', 'base_link')
        self.node.declare_parameter('child_frame_id', 'wrist_rgbd_camera_depth_optical_frame')
        self.node.declare_parameter('translation_x', 0.338)
        self.node.declare_parameter('translation_y', 0.45)
        self.node.declare_parameter('translation_z', 0.1)
        self.node.declare_parameter('rotation_x', 0.0)
        self.node.declare_parameter('rotation_y', 0.866)
        self.node.declare_parameter('rotation_z', -0.5)
        self.node.declare_parameter('rotation_w', 0.0)

        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self.node)

    def publish_static_transform(self) -> None:
        # Read the parameters
        header_frame_id = self.node.get_parameter('header_frame_id').value
        child_frame_id = self.node.get_parameter('child_frame_id').value
        translation_x = self.node.get_parameter('translation_x').value
        translation_y = self.node.get_parameter('translation_y').value
        translation_z = self.node.get_parameter('translation_z').value
        rotation_x = self.node.get_parameter('rotation_x').value
        rotation_y = self.node.get_parameter('rotation_y').value
        rotation_z = self.node.get_parameter('rotation_z').value
        rotation_w = self.node.get_parameter('rotation_w').value

        static_transform_stamped = TransformStamped()
        static_transform_stamped.header.stamp = self.node.get_clock().now().to_msg()
        static_transform_stamped.header.frame_id = header_frame_id
        static_transform_stamped.child_frame_id = child_frame_id
        
        static_transform_stamped.transform.translation.x = translation_x
        static_transform_stamped.transform.translation.y = translation_y 
        static_transform_stamped.transform.translation.z = translation_z 
        static_transform_stamped.transform.rotation.x = rotation_x  
        static_transform_stamped.transform.rotation.y = rotation_y
        static_transform_stamped.transform.rotation.z = rotation_z 
        static_transform_stamped.transform.rotation.w = rotation_w 
        
        # Publish the static transform
        self.broadcaster.sendTransform(static_transform_stamped)

    def spin(self) -> None:
        rclpy.spin(self.node)
        self.node.destroy_node()
        rclpy.shutdown()

def main(args=None) -> None:
    rclpy.init(args=args)
    static_transform_publisher = StaticTransformPublisher()
    static_transform_publisher.publish_static_transform()
    static_transform_publisher.spin()

if __name__ == '__main__':
    main()