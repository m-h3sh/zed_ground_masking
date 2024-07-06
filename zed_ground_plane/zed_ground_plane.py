import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from zed_interfaces.msg import PlaneStamped
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import cv2
import numpy as np
from scipy.spatial import ConvexHull
from cv_bridge import CvBridge
import math
 
class zedNODE(Node):
 
    def __init__(self):
        super().__init__('pubsub')
        self.bridge = CvBridge()
        self.coordslist = [] # list for mesh vertices
 
        #creating publishers and subscribers
        self.publisher_ = self.create_publisher(PointStamped, '/clicked_point', 1)
        self.final_img_publisher = self.create_publisher(Image, '/zed/masked_image', 1)
        self.subscription = self.create_subscription(PlaneStamped, '/zed/plane', self.listener_callback, 10)
        self.camerasub = self.create_subscription(Image, '/zed/zed_node/left/image_rect_color', self.camera_callack, 10)
        self.cinfosub = self.create_subscription(CameraInfo, '/zed/zed_node/left/camera_info', self.infocallback, 10)
        self.camerasub
        self.subscription
        self.cinfosub
 
    # getting the mesh vertices from zed
    def listener_callback(self, msg):
        vertices = msg.mesh.vertices
 
        for vertex in vertices:
            self.coordslist.append((vertex.x, vertex.y, vertex.z))
 
 
    def camera_callack(self, data):
        # publish point to detect plane on ground
        point = PointStamped
        point.point.x = 1.40 / math.cos(1.1868238913561)
        point.point.y = 0
        point.point.z = 0
        self.publisher_.publish(point)

        self.cvimage = self.bridge.imgmsg_to_cv2(data, "bgr8") # converting ROS image to cv image
        # self.fx = 530.931884765625
        # self.fy = 530.931884765625
        # self.cx = 641.9871826171875
        # self.cy = 366.1305847167969
 
        #converting from 3D coordinates to pixel coordinates
        uvlist = []
        for vertex in self.coordslist:
            Z = vertex[0]
            X = vertex[1]
            Y = vertex[2]
            print(X, Y, Z)
            u = ((X / Z) * self.fx) + self.cx
            v = ((Y / Z) * self.fy) + self.cy
            if (u > 0 and u < 1280 and v > 0 and v < 720):
                uvlist.append((1280 - int(u),720 - int(v)))   # image is 1280 x 720
 
 
 
        if (len(uvlist) != 0):
            hull = ConvexHull(uvlist) # convex hull of the mesh
            mask = np.zeros((720,1280,3), np.uint8) # black image for mask
            convexpoints = np.array([uvlist[i] for i in hull.vertices], dtype=np.int32)
 
            cv2.fillPoly(mask, [convexpoints], (255, 255, 255)) # creating a white and black mask
            mask = mask.astype(np.float32) / 255.0
            masked_image = np.uint8(mask*self.cvimage)
 
            imgmessage = self.bridge.cv2_to_imgmsg(masked_image, encoding="passthrough") # converting cv image to ROS image
            self.final_img_publisher.publish(imgmessage)
 
    # getting camera information
    def infocallback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
 
 
def main(args=None):
    rclpy.init(args=args)
 
    velpub = zedNODE()
    rclpy.spin(velpub)
    velpub.destroy_node()
    rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()