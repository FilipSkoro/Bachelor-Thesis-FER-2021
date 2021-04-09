#!/usr/bin/env python3

class Task():

	def __init__(self, name, num_robots, duration, location_X, location_Y):
		self.name = name
		self.num_robots = num_robots
		self.duration = duration
		self.location_X = location_X
		self.location_Y = location_Y

	def __repr__(self):
		return self.name

	#def to_ros_task(self):
