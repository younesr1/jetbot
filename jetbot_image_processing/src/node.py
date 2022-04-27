#!/usr/bin/python3
import torch
import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np

class ImageProcessor():
    def __init__(self):
        # Monocular Depth Members
        model_type = ("MiDaS_small", "DPT_Hybrid", "DPT_Large")[0]
        self.midas = torch.hub.load("intel-isl/MiDaS", model_type)
        self.device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
        self.midas.to(self.device)
        self.midas.eval()
        midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
        self.transform = midas_transforms.small_transform if model_type == "MiDaS_small" else midas_transforms.dpt_transform
        
        rospy.loginfo("Instantiated Monocular Depth Model")

        # ROS Members
        self.queue_size = 50
        self.rgb_image_topic = rospy.get_param(
            "/jetbot_image_processing/rgb_image_topic")
        self.raw_depth_topic = rospy.get_param(
            "/jetbot_image_processing/raw_depth_topic")
        self.colorized_depth_topic = rospy.get_param(
            "/jetbot_image_processing/colorized_depth_topic")
        rospy.Subscriber(self.rgb_image_topic, Image, self.callback)
        self.colorized_depth_publisher = rospy.Publisher(self.colorized_depth_topic, Image, queue_size=self.queue_size)
        self.raw_depth_publisher = rospy.Publisher(self.raw_depth_topic, Image, queue_size=self.queue_size)



    def callback(self, rgb_image):
        raw_depth, colorized_depth = self.monodepth(self.img_msg_to_cv2(rgb_image))
        self.colorized_depth_publisher.publish(self.cv2_to_img_msg(colorized_depth, rgb_image.header)) #(self.bridge.cv2_to_imgmsg(colorized_depth, "bgr8"))
        rospy.loginfo("Published colorized depth")
        self.raw_depth_publisher.publish(self.cv2_to_img_msg(raw_depth, rgb_image.header)) # (self.bridge.cv2_to_imgmsg(raw_depth, "32FC1"))
        rospy.loginfo("Publushed raw depth")


    def monodepth(self, rgb_image):
        input_batch = self.transform(rgb_image).to(self.device)
        with torch.no_grad():
            prediction = self.midas(input_batch)
            prediction = torch.nn.functional.interpolate(
                prediction.unsqueeze(1),
                size=rgb_image.shape[:2],
                mode="bicubic",
                align_corners=False,
            ).squeeze()
        raw_depth = prediction.cpu().numpy()
        colorized_depth = cv2.applyColorMap(cv2.normalize(raw_depth, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U), cv2.COLORMAP_MAGMA)
        return raw_depth, colorized_depth

    
    def cv2_to_img_msg(self, img, header):
        ret = Image()
        ret.header = header
        ret.height, ret.width = img.shape[:2]
        if  len(img.shape) == 2 and img.dtype == np.float32:
            ret.encoding = "mono8"
        elif img.shape[2] == 3 and img.dtype == np.uint8:
            ret.encoding = "bgr8"
        else:
            rospy.logerr("Unrecognized image format in cv2_to_img_msg")
        ret.is_bigendian = img.dtype.byteorder == '>'
        ret.data = img.tostring()
        ret.step = len(ret.data) // ret.height
        return ret
    
    def img_msg_to_cv2(self, img):
        return np.frombuffer(img.data, dtype=np.uint8).reshape(img.height, img.width, -1)


def main():
    rospy.init_node("jetbot_image_processing")

    rospy.loginfo("Starting Jetbot Image Processing Node")

    ImageProcessor()

    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
