#!/usr/bin/python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageProcessor():
    def __init__(self):
        self.queue_size = 10

        self.pub = rospy.Publisher("/camera/image/raw", Image, queue_size=self.queue_size)

        self.bridge = CvBridge()

    def run(self):
        cap = cv2.VideoCapture('/home/younes/Documents/jetbot_ws/src/jetbot/jetbot_image_processing/src/test_drone.mp4')
        rate = rospy.Rate(30)
        assert cap.isOpened()
        while not rospy.is_shutdown():
            ret, frame = cap.read()
            if not ret:
                rospy.loginfo("Video Ended")
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                continue
            else:
                rospy.loginfo("Publishing frame")
                self.pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))                
            rate.sleep()


def main():
    rospy.init_node("video_pub")

    proc = ImageProcessor()
    proc.run()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
