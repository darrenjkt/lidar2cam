#!/usr/bin/env python

import math
import numpy as np
import rospy
import tf2_ros
import geometry_msgs.msg
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs import point_cloud2 as pc2
from image_geometry import PinholeCameraModel
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import CameraInfo,Image, PointCloud2


class LidarImageProjection:
	def __init__(self, img_topic, pcl_topic, caminfo_topic):
		self.sub_camintrinsics = rospy.Subscriber(caminfo_topic, CameraInfo, self.get_intrinsics, queue_size=1)

		# slop is the delay (in seconds) with which msgs can be synchronised
		self.sub_img_pcl = ApproximateTimeSynchronizer([Subscriber(img_topic, Image), Subscriber(pcl_topic, PointCloud2)], queue_size=5, slop=0.1)
		self.sub_img_pcl.registerCallback(self.img_pcl_callback)

		# Publishers
		self.img_pub = rospy.Publisher(img_topic + '/projected_pcl', Image, queue_size=10)

		# transforms and camera model
		self.camera = {}
		self.caminfo_obtained = False
		self.bridge = CvBridge()
		self.trans = None
		self.listener = None
		self.tfBuffer = None
		self.tf_obtained = False

	def img_pcl_callback(self, img_msg, pcl_msg):
		if self.tf_obtained == False:
			self.get_tf(img_msg, pcl_msg)
		else:
			try:
				cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
				img_projection = cv_image.copy()
				height, width = img_projection.shape[:2]
			except CvBridgeError as e:
				print(e)

			# Transform pointcloud into camera frame
			transformed_pcl = do_transform_cloud(pcl_msg, self.trans)

			# Get uv coordinates of xyz points and draw on image
			dist_coeff = self.camera['model'].distortionCoeffs()
			camera_matrix = np.array(self.camera['model'].intrinsicMatrix()) 
			
			
			for p in pc2.read_points(transformed_pcl, skip_nans=True):
				# Relationship between pixels and camera coordinates is
				# u = f_x*X/Z + u_0, v = f_y*Y/Z +v_0
				tmp_x = p[0]/p[2]
				tmp_y = p[1]/p[2]
				tmp_z = p[2]
				dist = (p[0]*p[0] + p[1]*p[1] + p[2]*p[2])**0.5
				
				# Apply lens distortion
				# TODO: Improve speed. This process of applying distortion reduces FPS from
				# 10 FPS -> 1 FPS
				r2 = tmp_x*tmp_x + tmp_y*tmp_y		
				r1 = r2**0.5
				a0 = math.atan(r1)
				a1 = a0 * (1 + dist_coeff[0]*(a0**2) \
						+ dist_coeff[1]*(a0**4) \
						+ dist_coeff[2]*(a0**6) \
						+ dist_coeff[3]*(a0**8))

				imgplane_x = (a1/r1) * tmp_x
				imgplane_y = (a1/r1) * tmp_y
				
				u = camera_matrix[0][0]*imgplane_x + camera_matrix[0][2]
				v = camera_matrix[1][1]*imgplane_y + camera_matrix[1][2]

				# Check if u,v points are outside the height and width of image or have negative values
				if (v >= 0 and v < height and u >= 0 and u < width \
					and tmp_z > 0 and abs(tmp_x) <= 1.35):
					cv2.circle(img_projection, (int(u),int(v)), 1, 255, -1)

			self.img_pub.publish(self.bridge.cv2_to_imgmsg(img_projection, "bgr8"))

	def get_tf(self, img_msg, pcl_msg, tf_cache_duration=2.0):
		lidar_frame_id = pcl_msg.header.frame_id
		cam_frame_id = img_msg.header.frame_id

		self.tfBuffer = tf2_ros.Buffer(rospy.Duration(tf_cache_duration))
		self.listener = tf2_ros.TransformListener(self.tfBuffer)
		try:
			# lookupTransform(child_frame, parent_frame) i.e. here it is lidar frame to camera
			self.trans = self.tfBuffer.lookup_transform(cam_frame_id, lidar_frame_id, rospy.Time())
			self.tf_obtained = True
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			pass

  	def get_intrinsics(self, caminfo_msg):
  		if self.caminfo_obtained == False:
	  		self.camera['model'] = PinholeCameraModel()
			self.camera['model'].fromCameraInfo(caminfo_msg)	
			self.camera['height'] = caminfo_msg.height
			self.camera['width'] = caminfo_msg.width
			self.caminfo_obtained = True


if __name__ == '__main__':
	rospy.init_node('lidar2cam_node', anonymous=True)   
	cam_topic = rospy.get_param("~cam_topic")
	pcl_topic = rospy.get_param("~pcl_topic")

	img_topic = cam_topic + '/image_color'
	caminfo_topic = cam_topic + '/camera_info'
	converter = LidarImageProjection(img_topic, pcl_topic, caminfo_topic)
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		cv2.destroyAllWindows()