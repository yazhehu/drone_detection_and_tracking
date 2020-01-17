#!/usr/bin/env python

def IOU(xmin1, ymin1, xmax1, ymax1, xmin2, ymin2, xmax2, ymax2):
	left_x = max(xmin1, xmin2)
	left_y = max(ymin1, ymin2)
	right_x = min(xmax1, xmax2)
	right_y = min(ymax1, ymax2)
	
	if (left_x >= right_x) or (left_y >= right_y):
		return 0
	else:
		width = right_x - left_x
		length = right_y - left_y
		area = width * length
		union = (xmax1 - xmin1) * (ymax1 - ymin1) + (xmax2 - xmin2) * (ymax2 - ymin2) - area
		return area/union
	
