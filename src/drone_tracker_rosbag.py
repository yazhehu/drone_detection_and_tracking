#!/usr/bin/env python
import message_filters
import cv2
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tracker.msg import BoundingBoxes

def callback(box, image):
	# initialize at the first time, then tracking
	global key
	print("debug1")
	if key == 0:
		xmin = box.bounding_boxes[0].xmin
		xmax = box.bounding_boxes[0].xmax
		ymin = box.bounding_boxes[0].ymin
		ymax = box.bounding_boxes[0].ymax
		bbox = (xmin, ymin, xmax-xmin, ymax-ymin)
		cv_image = bridge.imgmsg_to_cv2(image, "bgr8")
		global tracker
		ok = tracker.init(cv_image, bbox)
		key += 1


def callback_track(image):
	global key
	print("debug2")
	if key > 0:
		global tracker
		cv_image = bridge.imgmsg_to_cv2(image, "bgr8")

		# Start timer
		timer = cv2.getTickCount()
		ok, bbox = tracker.update(cv_image)
		time_spent = cv2.getTickCount() - timer;
		p1 = (int(bbox[0]), int(bbox[1]))
		p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
		cv2.rectangle(cv_image, p1, p2, (255,0,0), 2, 1)

		# publish 
		ros_image = bridge.cv2_to_imgmsg(cv_image, "bgr8")
		pub.publish(ros_image)
		time_total = cv2.getTickCount() - timer;
		print('tracking:', key)
		print('time_tracking:', time_spent)
		print('time_total:', time_total)
		key += 1


def tracker_setup():
	# Set up tracker.
    # Instead of MIL, you can also use
	tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'GOTURN', 'MOSSE', 'CSRT']
	tracker_type = tracker_types[4]

	if tracker_type == 'BOOSTING':
		tracker = cv2.TrackerBoosting_create()
	if tracker_type == 'MIL':
		tracker = cv2.TrackerMIL_create()
	if tracker_type == 'KCF':
		tracker = cv2.TrackerKCF_create()
	if tracker_type == 'TLD':
		tracker = cv2.TrackerTLD_create()
	if tracker_type == 'MEDIANFLOW':
		tracker = cv2.TrackerMedianFlow_create()
	if tracker_type == 'GOTURN':
		tracker = cv2.TrackerGOTURN_create()
	if tracker_type == 'MOSSE':
		tracker = cv2.TrackerMOSSE_create()
	if tracker_type == "CSRT":
		tracker = cv2.TrackerCSRT_create()
	
	return tracker

def listener():
	try:
		box_sub = message_filters.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, queue_size=1)
		#image_sub = message_filters.Subscriber('/usb_cam/image_raw', Image, queue_size=None)
		## playing rosbag
		image_sub = message_filters.Subscriber('/image_raw', Image, queue_size=1)

		ts = message_filters.ApproximateTimeSynchronizer([box_sub, image_sub], 3, 0.1)
		ts.registerCallback(callback)
	except:
		pass
	## tracking
	## playing rosbag
	rospy.Subscriber("/image_raw", Image, callback_track, queue_size=1, buff_size=2**24)
	rospy.spin()

if __name__ == '__main__' :
	bridge = CvBridge()
	rospy.init_node('drone_tracker_node', anonymous=True)
	pub = rospy.Publisher('/track_drone/image', Image, queue_size=1)
	print("init drone_tracker_node")
	key = 0
	tracker = tracker_setup()
	listener()
	
