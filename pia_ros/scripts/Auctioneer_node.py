#!/usr/bin/env python3

from Task import Task
from Bid import Bid
import rospy
import copy
from pia_ros.srv import ProvideTauc, ProvideTaucResponse, DeclareWinner, DeclareEnd, DeclareEndResponse, TightenSchedule

import matplotlib.pyplot as plt

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

		robots = []
		self.ns = []
		self.robot_name_location = {}

		robots = rospy.get_param("/agents")

		for r in robots:
			self.ns.append(r['name'].lower())

			robot_location = []
			robot_location_x = []
			robot_location_y = []

			robot_location = r['location'].split(", ")
			robot_location_x.append(float(robot_location[0]))
			robot_location_y.append(float(robot_location[1]))

			robot_location.clear()

			robot_location.append(robot_location_x)
			robot_location.append(robot_location_y)
			self.robot_name_location.update({r['name']: robot_location})

	def task_to_object(self):
		# This function creates and returns object of a type Task by using paramteres
		# available on server under name /tasks. Server paramateres are names and attributes 
		# uploaded by pia_ros.launch file from Tasks.yaml file.

		tasks = rospy.get_param("/tasks")

		for i in range(len(tasks)):
			location = tasks[i]["location"].split(", ")
			self.tasks.append(Task(tasks[i]["name"],tasks[i]["resources"],tasks[i]["duration"],tasks[i]["precedence"],tasks[i]["earliest_start_time"],tasks[i]["latest_finish_time"],location[0],location[1]))

		self.plot_tasks = self.tasks[:]

	def sort_Tasks(self):
		# This function sorts tasks loaded from Task.yaml file to three different lists: TF, TL and TH.
		# Tasks which don't have any task named in the attribute 'precedence' are tasks with the highest
		# priority and are put in the TF list. Tasks that have one or more tasks named in the attribute
		# 'precedence' are put in the TL list if their precedence tasks are already in the TF list. Finally
		# tasks left in the list self.tasks are tasks with the lowest priority and are put in the TH list.

		self.TF = []
		self.TL = []
		self.TH = []

		for i in range(2):
			#print(self.tasks)
			for task in self.tasks:
				#print(task)
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

		print(f'Tasks TF: {self.TF}')
		print(f'Tasks TL: {self.TL}')
		print(f'Tasks TH: {self.TH}\n')
		print(f"=========================================\n")

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
			for tl in self.TL:
				if len(tl.precedence) == 4:
					for tf in self.TF:
						if tl.precedence == tf.name:
							self.TH.append(tl)
							self.TL.remove(tl)
							self.TL.insert(0,tf)
							self.TF.remove(tf)
						else:
							continue
				elif len(tl.precedence) > 4:
					tl_pred = tl.precedence.split(", ")
					a = False
					for tl_p in tl_pred:
						for tf in self.TF:
							if tl_p == tf.name:
								self.TL.insert(0,tf)
								self.TF.remove(tf)
								a = True
							else:
								continue

					if a:
						self.TH.append(tl)
						self.TL.remove(tl)
					else:
						continue

			if self.TF:
				for th in self.TH:
					if len(th.precedence) == 4:
						for tf in self.TF:
							if th.precedence == tf.name:
								self.TL.append(tf)
								self.TF.remove(tf)
							else:
								continue
					elif len(th.precedence) > 4:
						th_pred = th.precedence.split(", ")
						for th_p in th_pred:
							for tf in self.TF:
								if th_p == tf.name:
									self.TL.append(tf)
									self.TF.remove(tf)
							else:
								continue

			for tf in self.TF:
				self.TH.append(tf)

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
			control = False
			for task in self.Tauc:
				TaucMsg.append(task.to_msg())

			for name in self.ns:
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
					control = True
					print("Bidder name: {0}".format(self.resp1.robot_name),end='\n')
					print("Bid: {0}".format(self.resp1.bid.name),end='\n')
					print("Time: {0} seconds\n".format(self.resp1.time))
				else:
					print(f"{self.resp1.robot_name} couldn't bid for any task at this moment.\n")

			if not control:
				print("No one could bid for rest of tasks on auction :(\n")
				print("Auctioneer is going to remove those tasks from auction.")
				self.Tauc.clear()

			self.choose_bid()

			if self.Tauc:
				print(f"=========================================\n")
				print(f"Tasks still left on auction: {self.Tauc}\n")
			elif self.TL and self.TH and not self.Tauc:
				for name in self.ns:
					rospy.wait_for_service(f'/{name}/tighten_schedule')
					try:
						tighten_schedule = rospy.ServiceProxy(f'/{name}/tighten_schedule', TightenSchedule)
						tighten_schedule(True)
					except rospy.ServiceException as exc:
						print(end='')

				self.define_Tauc()

				print(f"=========================================\n")
				print(f"Tasks TL on auction: {self.Tauc}\n")
			elif self.TH and not self.Tauc:
				for name in self.ns:
					rospy.wait_for_service(f'/{name}/tighten_schedule')
					try:
						tighten_schedule = rospy.ServiceProxy(f'/{name}/tighten_schedule', TightenSchedule)
						tighten_schedule(True)
					except rospy.ServiceException as exc:
						print(end='')

				self.define_Tauc()

				print(f"=========================================\n")
				print(f"Tasks TH on auction: {self.Tauc}\n")

		print("=========================================\n")
		for name in self.ns:
			rospy.wait_for_service(f'/{name}/declare_end')
			try:
				self.resp2 = DeclareEndResponse()
				declare_end = rospy.ServiceProxy(f'/{name}/declare_end', DeclareEnd)
				self.resp2 = declare_end(True)
				for r_name in self.robot_name_location:
					if r_name == name.capitalize():
						for tsk in self.resp2.schedule:
							t = Task.to_task(tsk)
							self.robot_name_location[r_name][0].append(float(t.location_X))
							self.robot_name_location[r_name][1].append(float(t.location_Y))
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
				for bid in self.bids:
					if bid.bid.name == task_name:
						if bid.time < task_time:
							rospy.wait_for_service(f'/{robot_name.lower()}/declare_winner')
							try:
								declare_winner = rospy.ServiceProxy(f'/{robot_name.lower()}/declare_winner', DeclareWinner)
								declare_winner(False)
							except rospy.ServiceException as exc:
								print(end='')

							task_time = copy.deepcopy(bid.time)
							robot_name = copy.deepcopy(bid.robot_name)
							task_name = copy.deepcopy(bid.bid.name)
							self.bids.remove(bid)
						else:
							rospy.wait_for_service(f'/{bid.robot_name.lower()}/declare_winner')
							try:
								declare_winner = rospy.ServiceProxy(f'/{bid.robot_name.lower()}/declare_winner', DeclareWinner)
								declare_winner(False)
							except rospy.ServiceException as exc:
								print(end='')

							self.bids.remove(bid)
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

	def plot_robots_and_tasks(self):
		# This function plots robots schedule in the coordinate system.

		plt.scatter(self.robot_name_location['Robot1'][0],self.robot_name_location['Robot1'][1],color='b',label="Robot1 schedule")
		plt.plot(self.robot_name_location['Robot1'][0],self.robot_name_location['Robot1'][1])

		plt.scatter(self.robot_name_location['Robot2'][0],self.robot_name_location['Robot2'][1],color='r',label="Robot2 schedule")
		plt.plot(self.robot_name_location['Robot2'][0],self.robot_name_location['Robot2'][1])

		plt.scatter(self.robot_name_location['Robot3'][0],self.robot_name_location['Robot3'][1],color='g',label="Robot3 schedule")
		plt.plot(self.robot_name_location['Robot3'][0],self.robot_name_location['Robot3'][1])

		plt.scatter(self.robot_name_location['Robot4'][0],self.robot_name_location['Robot4'][1],color='y',label="Robot4 schedule")
		plt.plot(self.robot_name_location['Robot4'][0],self.robot_name_location['Robot4'][1])

		plt.xlabel("X coordinate")
		plt.ylabel("Y coordinate")
		plt.grid(True)
		plt.title("Robots and Tasks positions")
		plt.legend()
		plt.show()

if __name__ == "__main__":
	# This is 'main function' used for initializing node and creating
	# object of a type Auctioneer. It calls function for creating Task
	# objects and Simple Iterated Auction function.

	rospy.init_node('Auctioneer')

	try:
		A = Auctioneer()
		A.task_to_object()

		A.sIA_function()
		A.plot_robots_and_tasks()

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