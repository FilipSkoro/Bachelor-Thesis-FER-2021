#!/usr/bin/env python3

from Bidder_node import Robot
from Task_node import Task
import rospy

class Auctioneer():

	def __init__(self):
		print("Stvorio sam se!")
		self.tasks = []
		self.n = 0

	def agent_to_object(self, file):
		lines = file.readlines()
		attr = []
		R = []
		lines = lines[:lines.index("tasks:\n")]
		for line in lines:
			if line == "\n" or line == "agents:\n":
				continue
			else:
				pom = line.split(": ")
				if pom[0] == '    location':
					pom2 = pom[1].split(", ")
					attr.append(pom2[0])
					attr.append(pom2[1])
					R.append(Robot(attr[0], attr[1], attr[2], attr[3]))
					attr.clear()
				else:
					attr.append(pom[1])

		return R

	def task_to_object(self, file):
		file.seek(0)
		lines = file.readlines()
		attr = []
		lines = lines[lines.index("tasks:\n"):]
		for l in lines:
			if l == "\n" or l == "tasks:\n":
				continue
			else:
				pom = l.split(": ")
				if pom[0] == '    location':
					pom2 = pom[1].split(", ")
					attr.append(pom2[0])
					attr.append(pom2[1])
					self.tasks.append(Task(attr[0], attr[1], attr[2], attr[3], attr[4]))
					attr.clear()
					self.n = self.n + 1
				else:
					attr.append(pom[1])

	def sIA_function(self, R):
		self.TS = []
		self.TF = []
		self.F = []
		self.PC = []
		self.Tauc = []

		self.TF = self.tasks[:]
		#while len(self.TS) < self.n:
		self.Tauc = self.TF[:]
		self.Tauc_pub.publish(self.Tauc)
		#R[0].ModifiedSSI(self.Tauc)
		#R[1].ModifiedSSI(self.Tauc)
			#ReceiveFinishTimes(R,F)

		#return F

## Input: Precedence graph GP = (T,E), robots R
##	TS = ∅, TF = free(T)
##	TL = free(T \ TF), TH = T \ (TF ∪ TL)
##	Initialize arrays F, PC to zero
##	while |TS| < n do
##		c = max t ∈ TL (prio(t))
##		Tauct = {t ∈ TF : prio(t) ≥ c}
##		ModifiedSSI(Tauct, PC)
##		ReceiveFinishTimes(R,F)
##		for all t ∈ Tauct do
##			UpdatePrecGraph(t, PC, GP)
##		end for
##	end while
##	return F
