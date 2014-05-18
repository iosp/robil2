#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, CogniTeam, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import robil_monitor_lib
import rospy
from ibus import exception

class RobilLibProxyVars:
	"""
	Local constant variables
	"""
	@staticmethod
	def script_name():
		return "${SCRIPT_NAME}"


class RobilLibWrapper:
	"""
	Wrapper of the internal methods
	"""
	def invoke(self, function_name, *args):
		# Construct arguments
		args_tuple = (RobilLibProxyVars.script_name(), function_name)

		# Convert all arguments to string
		for p in args: args_tuple += (str(p),)

		# Invoke cpp method
		return robil_monitor_lib.internal_function(args_tuple)

	def topic(self, topic_name):
		# Construct arguments
		args_tuple = (RobilLibProxyVars.script_name(), topic_name)

		# Invoke cpp method
		return robil_monitor_lib.internal_topic(args_tuple)


class ValidatorWrapper:
	"""
	Statement validation wrapper
	"""
	@staticmethod
	def internal_validation(validation_state, description):
		# Construct arguments
		args_tuple = (RobilLibProxyVars.script_name(), validation_state, description)

		# Invoke cpp method to inform about validation result
		robil_monitor_lib.internal_validation(args_tuple)

	@staticmethod
	def is_true(bool_expression, description = ""):
		if (bool_expression):
			# Validation succeeded
			ValidatorWrapper.internal_validation(True, description)
		else:
			# Validation failed
			ValidatorWrapper.internal_validation(False, description)



robil_lib 	= RobilLibWrapper()
validate 	= ValidatorWrapper()

def topic(topic_name):
	return robil_lib.topic(str(topic_name))


# ########################
# User code goes below ###
# ########################

# Simple CPU usage monitor example

# core1 = robil_lib.topic("/diagnostic/cpu/usage/core1")
# core2 = robil_lib.topic("/diagnostic/cpu/usage/core2")
# core3 = robil_lib.topic("/diagnostic/cpu/usage/core3")
# core4 = robil_lib.topic("/diagnostic/cpu/usage/core4")

# cpu_threshold = robil_lib.topic("/robot/config/cpu_threshold")
# core_threshold = robil_lib.topic("/robot/config/core_threshold")

# average = f('average_algorithm', core1, core2, core3, core4)

# validate.is_true(average < cpu_threshold, "Cpu usage too high!")
# validate.is_true(core1 < core_threshold, "Core1 usage too high")
# validate.is_true(core2 < core_threshold, "Core2 usage too high")
# validate.is_true(core3 < core_threshold, "Core3 usage too high")
# validate.is_true(core4 < core_threshold, "Core4 usage too high")


