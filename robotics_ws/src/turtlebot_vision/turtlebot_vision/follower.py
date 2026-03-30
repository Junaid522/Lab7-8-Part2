import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np


class LineFollower(Node):

    def __init__(self):
        super().__init__('line_follower')

        self.bridge = CvBridge()
        self.twist = Twist()

        # subscribe to the raw camera feed
        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        # velocity commands go here
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # debug image so we can see what the robot sees
        self.dbg_pub = self.create_publisher(Image, '/camera/line_debug', 10)

        # driving parameters – tweak these if the robot overshoots corners
        self.speed = 0.22        # m/s forward
        self.steer_div = 80.0    # larger = gentler turns

    # ------------------------------------------------------------------ #

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]

        # only look at the bottom third of the image; the top is mostly sky
        roi_top = int(2 * h / 3)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)
        mask[:roi_top, :] = 0   # blank out everything above the ROI line

        # build the debug overlay once so we can annotate it below
        debug = self._make_debug_image(frame, mask, roi_top, w)

        M = cv2.moments(mask)
        cmd = Twist()

        if M['m00'] > 0:
            # centroid of the detected line blob
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            error = cx - w / 2   # positive = line is to the right of centre

            cmd.linear.x  = self.speed
            cmd.angular.z = -error / self.steer_div

            # draw steering indicator on debug image
            cv2.circle(debug, (cx, cy), 10, (0, 0, 255), -1)
            cv2.line(debug, (w // 2, h), (cx, cy), (0, 0, 255), 2)
            self._put_text(debug, f'Error : {error:+.1f} px', 30)
            self._put_text(debug, f'Speed : {cmd.linear.x:.2f} m/s', 60)
            self._put_text(debug, f'Steer : {cmd.angular.z:.3f} rad/s', 90)

            self.get_logger().info(
                f'cx={cx}  err={error:+.1f}  v={cmd.linear.x:.2f}  w={cmd.angular.z:.3f}')
        else:
            # lost the line – stop and warn
            cmd.linear.x  = 0.0
            cmd.angular.z = 0.0
            self._put_text(debug, 'NO LINE DETECTED', 30, colour=(0, 0, 255))
            self.get_logger().warn('Line lost – stopping robot')

        self.cmd_pub.publish(cmd)
        self._publish_debug(debug, msg)

    # ------------------------------------------------------------------ #
    # helpers

    def _make_debug_image(self, frame, mask, roi_top, w):
        dbg = frame.copy()

        # green tint over detected pixels
        overlay = np.zeros_like(frame)
        overlay[:, :, 1] = mask
        dbg = cv2.addWeighted(dbg, 0.7, overlay, 0.3, 0)

        # yellow line showing the ROI boundary
        cv2.line(dbg, (0, roi_top), (w, roi_top), (0, 255, 255), 2)
        return dbg

    def _put_text(self, img, text, y, colour=(255, 255, 255)):
        cv2.putText(img, text, (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, colour, 2)

    def _publish_debug(self, img, original_msg):
        out = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        out.header = original_msg.header
        self.dbg_pub.publish(out)


# ------------------------------------------------------------------ #

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()