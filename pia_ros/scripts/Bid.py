#!/usr/bin/env python3

class Bid():
    # Object created with a type of this class represents bid sent by bidder.
	# It has the same attributes as object of a type ProvideTaucResponse.

    def __init__(self, bid, time, robot_name):
        # This function initializes object of the class Bid with attributes bid of
        # a type Task, total schedule time and bidders name.

        self.bid = bid
        self.time = time
        self.robot_name = robot_name

    def __repr__(self):
        # This function overrides method for printing objects attributes and
		# instead it prints just name of the attribute of a type Task.

	    return self.bid.name