import rclpy 
from rclpy.node import Node
from std_msgs.msg import String
from shape_msgs.msg import MeshTriangle    
from geometry_msgs.msg import PointStamped
from zed_msgs.msg import PlaneStamped
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import cv2
import numpy as np
from scipy.spatial import ConvexHull
from cv_bridge import CvBridge
import math

DEBUG = 0
 
class zedNODE(Node):
 
    def __init__(self):
        super().__init__('pubsub')
        self.bridge = CvBridge()
        self.coordslist = [] # list for mesh vertices
        self.uvlist = [] # list for uv coordinates of mesh vertices
        self.triangles = [] # list storing the indices of the vertices of each triangle
        self.boundary_xyz = [] # list of boundary points in xyz coordinates
        self.boundary_uv = [] # list of boundary points in uv coordinates
        self.boundary_vertices = []
        self.MASK_THRESHOLD = 4.5/8

        #creating publishers and subscribers
        self.publisher_ = self.create_publisher(PointStamped, '/clicked_point', 1)
        self.final_img_publisher = self.create_publisher(Image, '/zed/masked_image', 1)
        self.subscription = self.create_subscription(PlaneStamped, '/zed/plane', self.listener_callback, 10)
        self.camerasub = self.create_subscription(Image, '/zed/zed_node/rgb/image_rect_color', self.camera_callback, 10)
        self.cinfosub = self.create_subscription(CameraInfo, '/zed/zed_node/left/camera_info', self.infocallback, 10)
        self.camerasub
        self.subscription
        self.cinfosub

    def listener_callback(self, msg):
        # getting the mesh vertices from zed
        if not msg.mesh.vertices or not msg.mesh.triangles:
            self.get_logger().info(f'No data received from ZED!')
            return

        # DO NOT REMOVE
        self.coordslist = []
        # triangles are in the IMAGE coordinates, ie (u,v)
        self.triangles = []
        vertices = msg.mesh.vertices
        triangles = msg.mesh.triangles
        size = 0
        for vertex in vertices:
            self.coordslist.append([vertex.x, vertex.y, vertex.z])
        for triangle in triangles:
            uv_points = np.array([self.xyz_to_uv(vertices[index]) for index in triangle.vertex_indices])
            self.triangles.append(uv_points)

    def xyz_to_uv(self, vertex):
        # converting from 3D coordinates to pixel coordinates
        # dont ask me why
        Z = vertex.x
        X = vertex.y
        Y = vertex.z
        u = ((X / Z) * self.fx) + self.cx
        v = ((Y / Z) * self.fy) + self.cy
        return [int(self.cvimage.shape[1] - u),int(self.cvimage.shape[0] - v)]


    def camera_callback(self, data):
        # publish point to detect plane on ground
        # Adjust this point according to the height of the camera
        # while testing, by echoing /clicked_point and clicking in rviz2
        point = PointStamped()
        point.point.x = 1.9938502311706543#1.841294527053833 # 1.7975525856018066
        point.point.y = 0.09294271469116211 # 0.12550228834152222 # 0.09784013032913208
        point.point.z = -1.1642165184020996 # -1.423006534576416 # 0.011430501937866211
        point.header.frame_id = "zed_left_camera_frame"
        self.publisher_.publish(point)

        self.cvimage = self.bridge.imgmsg_to_cv2(data, "bgr8") # converting ROS image to cv image

        # First masking out the top fraction of the image
        # If no plane found, we publish this bottom part of the image 
        # without ground plane masking
        cutoff_height = int(self.cvimage.shape[0] * self.MASK_THRESHOLD)
        self.cvimage[:cutoff_height, :] = 0
 
        #TODO: set this parameter
        if (len(self.triangles) <= 10):
            self.get_logger().info(f'Bad plane detected, publishing bottom part of image')
            self.masked_image = self.cvimage

        else:
            print(len(self.triangles))
            self.get_logger().info(f'Plane detected with {len(self.boundary_uv)} points!')
            mask = np.zeros(self.cvimage.shape, np.uint8) # black image for mask
            # this took some time to get right, but drawContours is the worst
            # function in existence. 
            # deep breath....the triangles are a LIST of numpy arrays of points which are a list 
            # of two coordinates 
            cv2.drawContours(mask, self.triangles, -1, (255,255,255), -1)
            mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
            self.masked_image = cv2.bitwise_and(self.cvimage, self.cvimage, mask=mask)
            if (DEBUG):
                cv2.imshow("original image", self.cvimage)
                cv2.imshow("mask", mask)
                cv2.imshow("masked image", self.masked_image)
                cv2.waitKey(1)

        imgmessage = self.bridge.cv2_to_imgmsg(self.masked_image, "rgb8") # converting cv image to ROS image
        self.final_img_publisher.publish(imgmessage)


    # getting camera information
    def infocallback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

        if (DEBUG):
            self.get_logger().info(f'Camera parameters are {self.fx, self.fy, self.cx, self.cy}')
 
 
def main(args=None):
    rclpy.init(args=args)
    velpub = zedNODE()
    rclpy.spin(velpub)
    velpub.destroy_node()
    rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()
