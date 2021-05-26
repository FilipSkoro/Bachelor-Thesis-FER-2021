#!/usr/bin/env python3

from pia_ros.msg import TaskMessage

class Task():
	# Object created with a type of this class represents task and its attributes
	# contained in the Tasks.yaml file.

	def __init__(self, name, resources, duration, precedence, earliest_start_time, latest_finish_time, location_X, location_Y):
		# This function initializes object of the class Task with attributes name,
		# task resources, task duration time, earliest and latest possible task finish times
		# and coordinates X and Y.

		self.name = name
		self.resources = resources
		self.duration = duration
		self.precedence = precedence
		self.earliest_start_time = earliest_start_time
		self.latest_finish_time = latest_finish_time
		self.location_X = location_X
		self.location_Y = location_Y

	def __repr__(self):
		# This function overrides method for printing objects attributes and
		# instead it prints just task name.

		return self.name

	def to_msg(self):
		# This function turns object of a type Task into object of a
		# type TaskMessage used as an attribute when calling rossservice
		# of a type ProvideTauc. It returns TaskMessage object.

		msg = TaskMessage()
		msg.name = self.name
		msg.resources = self.resources
		msg.duration = self.duration
		msg.precedence = self.precedence
		msg.earliest_start_time = self.earliest_start_time
		msg.latest_finish_time = self.latest_finish_time
		msg.location_X = self.location_X
		msg.location_Y = self.location_Y

		return msg

	@classmethod
	def to_task(cls, msg):
		# This is a class method that turns object of a type TaskMessage into 
		# object of a type Task. It returns Task object.

		return cls(msg.name, msg.resources, msg.duration, msg.precedence, msg.earliest_start_time, msg.latest_finish_time, msg.location_X, msg.location_Y)