#!/usr/bin/env python
import message_filters
import cv2
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tracker.msg import BoundingBoxes
from functions import IOU

def callback(box, image):
	# initialize at the first time, then tracking
	global tracker_number
	global key
	global bbox
	print("initialize tracker", key)
	xmin = box.bounding_boxes[0].xmin
	xmax = box.bounding_boxes[0].xmax
	ymin = box.bounding_boxes[0].ymin
	ymax = box.bounding_boxes[0].ymax
	bbox_init = (xmin, ymin, xmax-xmin, ymax-ymin)
	xmin2 = bbox[0]
	ymin2 = bbox[0] + bbox[2]
	xmax2 = bbox[1]
	ymax2 = bbox[1] + bbox[3]
	iou = IOU(xmin, ymin, xmax, ymax, xmin2, ymin2, xmax2, ymax2)
	if iou < 0.5:
		key = 0

	if key == 0:
		cv_image = bridge.imgmsg_to_cv2(image, "bgr8")
		print('bbox')
		print(bbox_init)
		global tracker
		tracker = tracker_setup(tracker_number)
		ok = tracker.init(cv_image, bbox_init)
		key = 1


def callback_track(image):
	global key
	global tracker
	global bbox
	if key > 0:
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
		#print('time_tracking:', time_spent)
		#print('time_total:', time_total)
		key += 1
		
		# terminate tracking after key frames
		if key > 100:
			key = 0
			


def tracker_setup(number):
	# Set up tracker.
    # Instead of MIL, you can also use
	tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'GOTURN', 'MOSSE', 'CSRT']
	tracker_type = tracker_types[number]

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

	box_sub = message_filters.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, queue_size=None)
	#image_sub = message_filters.Subscriber('/usb_cam/image_raw', Image, queue_size=None)
	## playing rosbag
	image_sub = message_filters.Subscriber('/usb_cam/image_raw', Image, queue_size=None)

	ts = message_filters.ApproximateTimeSynchronizer([box_sub, image_sub], 3, 0.1)
	ts.registerCallback(callback)

	## tracking
	## playing rosbag
	rospy.Subscriber("/usb_cam/image_raw", Image, callback_track, queue_size=1, buff_size=2**24)
	rospy.spin()

if __name__ == '__main__' :
	bridge = CvBridge()
	rospy.init_node('drone_tracker_node', anonymous=True)
	pub = rospy.Publisher('/track_drone/image', Image, queue_size=1)
	print("init drone_tracker_node")
	bbox = [0,0,0,0]
	key = 0
	tracker_number = 4
	listener()
	
