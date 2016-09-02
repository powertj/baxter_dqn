#!/usr/bin/env python

import math

def move_vertical(positions, direction):
 
	# Limb Lengths
	r1 = 0.37082 
	r2 = 0.37442 
	s1_offset = math.atan2(0.069,r1) # offset due to structure of baxter arm at the e1 joint

	# Find x and y coordinates of w1 (wrist) joint from current s1 and e1 joint angles
	x = r1*math.cos(positions["left_s1"]+s1_offset) \
			+r2*math.cos(positions["left_s1"]+positions["left_e1"]+s1_offset)
	y = r1*math.sin(positions["left_s1"]+s1_offset) \
			+r2*math.sin(positions["left_s1"]+positions["left_e1"]+s1_offset)
	
	# Change y coordinate
	if direction == 'd':
		y = 0.2065
	elif direction == 'u':
		y = 0.0


	#Recalculate and set desired s1 and e1 angles

	cosine_theta2 = (math.pow(x,2) + math.pow(y,2) - math.pow(r1,2) - math.pow(r2,2))/ \
				(2*r1*r2)

	positions["left_e1"] = math.atan2(math.sqrt(abs(1-math.pow(cosine_theta2,2)) \
						),cosine_theta2)
	k1 = r1 + r2*math.cos(positions["left_e1"])
	k2 = r2*math.sin(positions["left_e1"])
	
	positions["left_s1"] = math.atan2(y,x) - math.atan2(k2,k1) - s1_offset

	# Keep w1 such that grip is vertical
	positions["left_w1"] = math.pi/2.0 - positions["left_e1"] -  \
						positions["left_s1"]
	
	return positions
