#!/usr/bin/env python3

from Task import Task
from Bid import Bid
import rospy
import copy
from pia_ros.srv import ProvideTauc, ProvideTaucResponse, DeclareWinner, DeclareEnd

class Auctioneer():
	# Object created with a type of this class represents auctioneer and its attributes.
	# This class provides some useful methods and functions used for Iterated Auction with Prioritization algorithm.

	def __init__(self):
		# This function initializes object of the class Auctioneer
		# and creates list called tasks in which all of the tasks needed
		# for auction will be stored once they are loaded from server.

		print("\n======================================================")
		print("\n*** Iterated Auction with Prioritization algorithm ***")
		print("\n======================================================\n")
		self.tasks = []

	def task_to_object(self):
		# This function creates and returns object of a type Task by using paramteres
		# available on server under name /tasks. Server paramateres are names and attributes 
		# uploaded by pia_ros.launch file from Tasks.yaml file.

		tasks = rospy.get_param("/tasks")

		for i in range(len(tasks)):
			location = tasks[i]["location"].split(", ")
			self.tasks.append(Task(tasks[i]["name"],tasks[i]["resources"],tasks[i]["duration"],tasks[i]["precedence"],location[0],location[1]))

	def sort_Tasks(self):
		# This function sorts tasks loaded from Task.yaml file to three different lists: TF, TL and TH.
		# Tasks which don't have any task named in the attribute 'precedence' are tasks with the highest
		# priority and are put in the TF list. Tasks that have one or more tasks named in the attribute
		# 'precedence' are put in the TL list if their precedence tasks are already in the TF list. Finally
		# tasks left in the list self.tasks are tasks with the lowest priority and are put in the TH list.

		self.TF = []
		self.TL = []
		self.TH = []

		for task in self.tasks:
			if task.precedence == '/':
				self.TF.append(task)
				self.tasks.remove(task)
			else:
				continue

		for task in self.tasks:
			if len(task.precedence) == 4:
				for t in self.TF:
					if task.precedence == t.name:
						self.TL.append(task)
						self.tasks.remove(task)
					else:
						continue
			elif len(task.precedence) > 4:
				tf_tasks = task.precedence.split(", ")
				for t in self.TF:
					if tf_tasks[0] == t.name:
						self.TL.append(task)
						self.tasks.remove(task)
					else:
						continue

		self.TH = self.tasks[:]

	def define_Tauc(self):
		# This function represents main algorithm for iterated auction with prioritization.
		# After each task have been separated in one of the three possible lists this function
		# decides which tasks are going to be auctioned. Tasks from TF list which have graph made
		# from tasks from TL and TH have the highest priority.

		if self.TF:
			for th in self.TH:
				if len(th.precedence) == 4:
					for tl in self.TL:
						if th.precedence == tl.name:
							if len(tl.precedence) == 4:
								for tf in self.TF:
									if tl.precedence == tf.name:
										self.Tauc.append(tf)
										self.TF.remove(tf)
									else:
										continue
							else:
								tl_predecessors = tl.precedence.split(", ")
								for tl_p in tl_predecessors:
									for tf in self.TF:
										if tl_p == tf.name:
											self.Tauc.append(tf)
											self.TF.remove(tf)
										else:
											continue
						else:
							continue
				else:
					th_predecessors = th.precedence.split(", ")
					for th_p in th_predecessors:
						for tl in self.TL:
							if th_p == tl.name:
								if len(tl.precedence) == 4:
									for tf in self.TF:
										if tl.precedence == tf.name:
											self.Tauc.append(tf)
											self.TF.remove(tf)
										else:
											continue
								else:
									tl_predecessors = tl.precedence.split(", ")
									for tl_p in tl_predecessors:
										for tf in self.TF:
											if tl_p == tf.name:
												self.Tauc.append(tf)
												self.TF.remove(tf)
											else:
												continue
							else:
								continue

		if self.TF:
			for tf in self.TF:
				for tl in self.TL:
					if len(tl.precedence) == 4:
						if tl.precedence == tf.name:
							self.TH.append(tl)
							self.TL.remove(tl)
						else:
							continue
					elif len(tl.precedence) > 4:
						tl_pred = tl.precedence.split(", ")
						for tl_p in tl_pred:
							if tl_p == tf.name:
								self.TH.append(tl)
								self.TL.remove(tl)
							else:
								continue
					else:
						self.Tauc.append(tf)
						self.TF.remove(tf)

			for tf in self.TF:
				self.TL.append(tf)

			self.TF.clear()

		elif self.TL and self.TH and not self.Tauc:
			self.Tauc = self.TL[:]
			self.TL.clear()

		elif self.TH and not self.Tauc:
			self.Tauc = self.TH[:]
			self.TH.clear()

	def sIA_function(self):
		# This function represents The Simple Itareted Auction algorithm. It provides
		# list of tasks that need to be assigned to the agents. After it recieves bids from
		# the agents it calls choose_bid function that decides which bids are accepted and which
		# are not.

		self.Tauc = []
		self.bids = []
		TaucMsg = []

		self.sort_Tasks()

		self.define_Tauc()

		print(f'Tasks TF on auction: {self.Tauc}\n')

		while self.Tauc:
			TaucMsg.clear()
			for task in self.Tauc:
				TaucMsg.append(task.to_msg())

			for name in ['robot1', 'robot2']:																# <=== ovo ce trebati pametnije napraviti - M
				rospy.wait_for_service(f'/{name}/provide_tauc')
				try:
					self.resp1 = ProvideTaucResponse()
					provide_tauc = rospy.ServiceProxy(f'/{name}/provide_tauc', ProvideTauc)
					self.resp1 = provide_tauc(TaucMsg)
					if self.resp1.time != 0:
						self.bids.append(Bid(Task.to_task(self.resp1.bid),self.resp1.time,self.resp1.robot_name))
				except rospy.ServiceException as exc:
					print("Service did not process request: " + str(exc))

				if self.resp1.time != 0:
					print("Bidder name: {0}".format(self.resp1.robot_name),end='\n')
					print("Bid: {0}".format(self.resp1.bid.name),end='\n')
					print("Time: {0} seconds\n".format(self.resp1.time))
				else:
					print(f"{self.resp1.robot_name} couldn't bid for any task at this moment.\n")

			self.choose_bid()

			if self.Tauc:
				print(f"=========================================\n")
				print(f"Tasks still left on auction: {self.Tauc}\n")
			elif self.TL and self.TH and not self.Tauc:
				self.define_Tauc()
				print(f"=========================================\n")
				print(f"Tasks TL on auction: {self.Tauc}\n")
			elif self.TH and not self.Tauc:
				self.define_Tauc()
				print(f"=========================================\n")
				print(f"Tasks TH on auction: {self.Tauc}\n")

		print("=========================================\n")
		for name in ['robot1', 'robot2']:																	# <=== ovo ce trebati pametnije napraviti - M
			rospy.wait_for_service(f'/{name}/declare_end')
			try:
				declare_end = rospy.ServiceProxy(f'/{name}/declare_end', DeclareEnd)
				declare_end(True)
			except rospy.ServiceException as exc:
				print(end='')

	def choose_bid(self):
		# This function declares which bids are accepted and which are not.
		# If there is only one bid for a certain task then it is surely going to be
		# accepted. If there are two or more bids for the same task, the bid with the lowest
		# schedule time will be accepted while other bids will be declined. Tasks are being removed
		# from the auction list after they have been assigned.

		while self.bids:
			robot_name = copy.deepcopy(self.bids[0].robot_name)
			task_name = copy.deepcopy(self.bids[0].bid.name)
			task_time = copy.deepcopy(self.bids[0].time)

			self.bids.remove(self.bids[0])

			if self.bids:
				for i in range(len(self.bids)):
					if self.bids[i].bid.name == task_name:
						if self.bids[i].time < task_time:
							rospy.wait_for_service(f'/{robot_name.lower()}/declare_winner')
							try:
								declare_winner = rospy.ServiceProxy(f'/{robot_name.lower()}/declare_winner', DeclareWinner)
								declare_winner(False)
							except rospy.ServiceException as exc:
								print(end='')

							task_time = copy.deepcopy(self.bids[i].time)
							robot_name = copy.deepcopy(self.bids[i].robot_name)
							self.bids.remove(self.bids[i])
						else:
							rospy.wait_for_service(f'/{self.bids[i].robot_name.lower()}/declare_winner')
							try:
								declare_winner = rospy.ServiceProxy(f'/{self.bids[i].robot_name.lower()}/declare_winner', DeclareWinner)
								declare_winner(False)
							except rospy.ServiceException as exc:
								print(end='')

							self.bids.remove(self.bids[i])
					else:
						continue

			rospy.wait_for_service(f'/{robot_name.lower()}/declare_winner')
			try:
				declare_winner = rospy.ServiceProxy(f'/{robot_name.lower()}/declare_winner', DeclareWinner)
				declare_winner(True)
			except rospy.ServiceException as exc:
				print(end='')

			self.remove_task(task_name)

	def remove_task(self, task_name):
		# This function removes task from the list of tasks that are being
		# auctioned after it has been assigned.

		for task in self.Tauc:
			if task.name == task_name:
				self.Tauc.remove(task)
			else:
				continue

if __name__ == "__main__":
	# This is 'main function' used for initializing node and creating
	# object of a type Auctioneer. It calls function for creating Task
	# objects and Simple Iterated Auction function.

	rospy.init_node('Auctioneer')

	try:
		A = Auctioneer()
		A.task_to_object()

		A.sIA_function()

	except rospy.ROSInterruptException: pass

##	============================================
##	*** Iterated Auction with Prioritization ***
##	============================================
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