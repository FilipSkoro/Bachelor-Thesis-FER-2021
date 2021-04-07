#!/usr/bin/env python3

from Task_node import Task
from math import sqrt
import copy
import rospy

class Robot():

	def __init__(self, name, velocity, location_X, location_Y):
		self.name = name
		self.velocity = velocity
		self.location_X = float(location_X)
		self.location_Y = float(location_Y)
		self.robot_schedule = []
		self.Tauc = []

	def ModifiedSSI(self, Tauc):
		#while len(Tauc) > 0:
		b = float('inf')
		bidding_schedule = []
		tbid = 0
		for t in Tauc:
			tmin = 0
			the_best_possible_schedule = []
			for i in range(len(self.robot_schedule) + 1):
				temp_schedule = copy.deepcopy(self.robot_schedule)
				temp_schedule.insert(i, t)
				total_schedule_time = 0
				total_schedule_time = self.getScheduleTime(temp_schedule)
				if total_schedule_time < tmin or tmin == 0:
					tmin = total_schedule_time
					the_best_possible_schedule = temp_schedule
				else:
					continue
			if tmin < b:
				b = tmin
				bidding_schedule = the_best_possible_schedule
				tbid = t
				print(tbid.name)
				print(tmin)
			else:
				continue

			#send_Bid(b, tbid)
		#return F

	def getScheduleTime(self, schedule):
		total_time = 0

		for t in schedule:
			total_time = total_time + float(t.duration)

		for i in range(len(schedule)):
			if i == 0:
				distance = sqrt(pow((float(schedule[i].location_X) - float(self.location_X)),2) + pow((float(schedule[i].location_Y) - float(self.location_Y)),2))
			else:
				distance = sqrt((float(schedule[i].location_X) - float(schedule[i-1].location_X))**2 + (float(schedule[i].location_Y) - float(schedule[i-1].location_Y))**2)

			total_time = total_time + distance/float(self.velocity)

		return total_time

##	Input: Set of tasks Tauct, earliest start times PC
##	Output: Vector of finishing times F
##	while |Tauct| > 0 do
##		b = ∞
##		tbid = ∅
##		for all t ∈ Tauct do
##			for all positions p in current schedule S do
##				Insert t in S at p to get S'
##				if isValid(S') and bid(S') < b then
##					b = bid(S'), tbid = t
##				end if
##			end for
##		end for
##		sendBid(b, tbid)
##		rwin, twin = receiveWinner()
##		if r = r win then
##			Insert t win in S at best valid position
##		end if
##		Tauct = Tauct \ twin
##	TightenSchedule(F)
##	return F
