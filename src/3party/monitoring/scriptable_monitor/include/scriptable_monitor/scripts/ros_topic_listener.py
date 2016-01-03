#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
#
# Revision $Id$

# ###############################################################################################################################################
# xxd -i ros_topic_listener.py | sed 's/unsigned/const/g' | sed 's/ros_topic_listener_py/ROS_TOPIC_LISTENER_SCRIPT/g' > RosTopicListenerPython.h
# ###############################################################################################################################################

# make sure we aren't using floor division
from __future__ import division, print_function

NAME='rostopic'

import cogni_topic_listener

import os
import sys
import math
import socket
import time
import traceback
import yaml
import xmlrpclib

from operator import itemgetter
from urlparse import urlparse

import genpy

import roslib.message
import rosgraph

import rospy

class ROSTopicException(Exception):
    """
    Base exception class of rostopic-related errors
    """
    pass
class ROSTopicIOException(ROSTopicException):
    """
    rostopic errors related to network I/O failures
    """
    pass

def _check_master():
    """
    Make sure that master is available
    :raises: :exc:`ROSTopicException` If unable to successfully communicate with master
    """
    try:
        rosgraph.Master('/rostopic').getPid()
    except socket.error:
        raise ROSTopicIOException("Unable to communicate with master!")
    
def _master_get_topic_types(master):
    try:
        val = master.getTopicTypes()
    except xmlrpclib.Fault:
        #TODO: remove, this is for 1.1
        sys.stderr.write("WARNING: rostopic is being used against an older version of ROS/roscore\n")
        val = master.getPublishedTopics('/')
    return val

def _sleep(duration):
    rospy.rostime.wallsleep(duration)


   
def msgevalgen(pattern):
    """
    Generates a function that returns the relevant field (aka 'subtopic') of a Message object
    :param pattern: subtopic, e.g. /x. Must have a leading '/' if specified, ``str``
    :returns: function that converts a message into the desired value, ``fn(Message) -> value``
    """
    if not pattern or pattern == '/':
        return None
    def msgeval(msg):
        # I will probably replace this with some less beautiful but more efficient
        try:
            return eval('msg'+'.'.join(pattern.split('/')))
        except AttributeError as e:
            sys.stdout.write("no field named [%s]"%pattern+"\n")
            return None
    return msgeval
    
def _get_topic_type(topic):
    """
    subroutine for getting the topic type
    :returns: topic type, real topic name and fn to evaluate the message instance
    if the topic points to a field within a topic, e.g. /rosout/msg, ``(str, str, fn)``
    """
    try:
        val = _master_get_topic_types(rosgraph.Master('/rostopic'))
    except socket.error:
        raise ROSTopicIOException("Unable to communicate with master!")

    # exact match first, followed by prefix match
    matches = [(t, t_type) for t, t_type in val if t == topic]
    if not matches:
        matches = [(t, t_type) for t, t_type in val if topic.startswith(t+'/')]
        # choose longest match
        matches.sort(key=itemgetter(0), reverse=True)
    if matches:
        t, t_type = matches[0]
        if t_type == rosgraph.names.ANYTYPE:
            return None, None, None
        return t_type, t, msgevalgen(topic[len(t):])
    else:
        return None, None, None

# NOTE: this is used externally by rxplot
    
def get_topic_type(topic, blocking=False):
    """
    Get the topic type.

    :param topic: topic name, ``str``
    :param blocking: (default False) block until topic becomes available, ``bool``
    
    :returns: topic type, real topic name and fn to evaluate the message instance
      if the topic points to a field within a topic, e.g. /rosout/msg. fn is None otherwise. ``(str, str, fn)``
    :raises: :exc:`ROSTopicException` If master cannot be contacted
    """
    topic_type, real_topic, msg_eval = _get_topic_type(topic)
    if topic_type:
        return topic_type, real_topic, msg_eval
    elif blocking:
        sys.stderr.write("WARNING: topic [%s] does not appear to be published yet\n"%topic)
        while not rospy.is_shutdown():
            topic_type, real_topic, msg_eval = _get_topic_type(topic)
            if topic_type:
                return topic_type, real_topic, msg_eval
            else:
                _sleep(0.1)
    return None, None, None

def get_topic_class(topic, blocking=False):
    """
    Get the topic message class
    :returns: message class for topic, real topic
      name, and function for evaluating message objects into the subtopic
      (or ``None``). ``(Message, str, str)``
    :raises: :exc:`ROSTopicException` If topic type cannot be determined or loaded
    """
    topic_type, real_topic, msg_eval = get_topic_type(topic, blocking=blocking)
    if topic_type is None:
        return None, None, None
    msg_class = roslib.message.get_message_class(topic_type)
    if not msg_class:
        raise ROSTopicException("Cannot load message class for [%s]. Are your messages built?"%topic_type)
    return msg_class, real_topic, msg_eval



class CallbackEcho(object):
    """
    Callback instance that can print callback data in a variety of
    formats. Used for all variants of rostopic echo
    """

    def __init__(self, topic, msg_eval=None, plot=False, filter_fn=None,
                 echo_clear=False, echo_all_topics=False,
                 offset_time=False, count=None,
                 field_filter_fn=None):
        """
        :param plot: if ``True``, echo in plotting-friendly format, ``bool``
        :param filter_fn: function that evaluates to ``True`` if message is to be echo'd, ``fn(topic, msg)``
        :param echo_all_topics: (optional) if ``True``, echo all messages in bag, ``bool``
        :param offset_time: (optional) if ``True``, display time as offset from current time, ``bool``
        :param count: number of messages to echo, ``None`` for infinite, ``int``
        :param field_filter_fn: filter the fields that are strified for Messages, ``fn(Message)->iter(str)``
        """
        if topic and topic[-1] == '/':
            topic = topic[:-1]
        self.topic = topic
        self.msg_eval = msg_eval
        self.plot = plot
        self.filter_fn = filter_fn

        self.prefix = ''
        self.suffix = '' if not plot else ''# same as YAML document separator, bug #3291
        
        self.echo_all_topics = echo_all_topics
        self.offset_time = offset_time

        # done tracks when we've exceeded the count
        self.done = False
        self.max_count = count
        self.count = 0

        # determine which strifying function to use
        if plot:
            #TODOXXX: need to pass in filter function
            self.str_fn = _str_plot
            self.sep = ''
        else:
            #TODOXXX: need to pass in filter function
            self.str_fn = self.custom_strify_message
            if echo_clear:
                self.prefix = '\033[2J\033[;H'

        self.field_filter=field_filter_fn
        
        # first tracks whether or not we've printed anything yet. Need this for printing plot fields.
        self.first = True

        # cache
        self.last_topic = None
        self.last_msg_eval = None

    def custom_strify_message(self, val, indent='', time_offset=None, current_time=None, field_filter=None, type_information=None):
        # ensure to print uint8[] as array of numbers instead of string
        if type_information and type_information.startswith('uint8['):
            val = [ord(x) for x in val]
        return genpy.message.strify_message(val, indent=indent, time_offset=time_offset, current_time=current_time, field_filter=field_filter)

    def callback(self, data, callback_args, current_time=None):
        """
        Callback to pass to rospy.Subscriber or to call
        manually. rospy.Subscriber constructor must also pass in the
        topic name as an additional arg
        :param data: Message
        :param topic: topic name, ``str``
        :param current_time: override calculation of current time, :class:`genpy.Time`
        """
        topic = callback_args['topic']
        type_information = callback_args.get('type_information', None)
        if self.filter_fn is not None and not self.filter_fn(data):
            return

        if self.max_count is not None and self.count >= self.max_count:
            self.done = True
            return
        
        try:
            msg_eval = self.msg_eval
            if topic == self.topic:
                pass
            elif self.topic.startswith(topic + '/'):
                # self.topic is actually a reference to topic field, generate msgeval
                if topic == self.last_topic:
                    # use cached eval
                    msg_eval = self.last_msg_eval
                else:
                    # generate msg_eval and cache
                    self.last_msg_eval = msg_eval = msgevalgen(self.topic[len(topic):])
                    self.last_topic = topic
            elif not self.echo_all_topics:
                return

            if msg_eval is not None:
                data = msg_eval(data)
                
            # data can be None if msg_eval returns None
            if data is not None:
                # NOTE: we do all prints using direct writes to sys.stdout, which works better with piping
                
                self.count += 1
                str_output = ""
				
                if self.offset_time:
                    str_output = self.prefix+self.str_fn(data, time_offset=rospy.get_rostime(),current_time=current_time, field_filter=self.field_filter, type_information=type_information)+self.suffix
                else:
                    str_output = self.prefix+self.str_fn(data,current_time=current_time, field_filter=self.field_filter, type_information=type_information)+self.suffix
                # print(str_output)
                cogni_topic_listener.internal_topic_update(topic, str_output)




                # if self.offset_time:
                #     sys.stdout.write(self.prefix+\
                #                      self.str_fn(data, time_offset=rospy.get_rostime(),
                #                                  current_time=current_time, field_filter=self.field_filter, type_information=type_information) + \
                #                      self.suffix + '\n')
                # else:
                #     sys.stdout.write(self.prefix+\
                #                      self.str_fn(data,
                #                                  current_time=current_time, field_filter=self.field_filter, type_information=type_information) + \
                #                      self.suffix + '\n')

                # we have to flush in order before piping to work
                sys.stdout.flush()
            # #2778 : have to check count after incr to set done flag
            if self.max_count is not None and self.count >= self.max_count:
                self.done = True

        except IOError:
            self.done = True
        except:
            # set done flag so we exit
            self.done = True
            traceback.print_exc()
            




def _rostopic_echo(topic, callback_echo, bag_file=None, echo_all_topics=False):
    """
    Print new messages on topic to screen.
    
    :param topic: topic name, ``str``
    :param bag_file: name of bag file to echo messages from or ``None``, ``str``
    """
    # we have to init a node regardless and bag echoing can print timestamps

    if bag_file:
        # initialize rospy time due to potential timestamp printing
        rospy.rostime.set_rostime_initialized(True)        
        _rostopic_echo_bag(callback_echo, bag_file)
    else:
        _check_master()
        rospy.init_node(NAME, anonymous=True)
        msg_class, real_topic, msg_eval = get_topic_class(topic, blocking=True)
        if msg_class is None:
            # occurs on ctrl-C
            return
        callback_echo.msg_eval = msg_eval

        # extract type information for submessages
        type_information = None
        if len(topic) > len(real_topic):
            subtopic = topic[len(real_topic):]
            subtopic = subtopic.strip('/')
            if subtopic:
                fields = subtopic.split('/')
                submsg_class = msg_class
                while fields:
                    field = fields[0].split('[')[0]
                    del fields[0]
                    index = submsg_class.__slots__.index(field)
                    type_information = submsg_class._slot_types[index]
                    if fields:
                        submsg_class = roslib.message.get_message_class(type_information)

        use_sim_time = rospy.get_param('/use_sim_time', False)
        sub = rospy.Subscriber(real_topic, msg_class, callback_echo.callback, {'topic': topic, 'type_information': type_information})

        if use_sim_time:
            # #2950: print warning if nothing received for two seconds

            timeout_t = time.time() + 2.
            while time.time() < timeout_t and \
                    callback_echo.count == 0 and \
                    not rospy.is_shutdown() and \
                    not callback_echo.done:
                _sleep(0.1)

            if callback_echo.count == 0 and \
                    not rospy.is_shutdown() and \
                    not callback_echo.done:
                sys.stderr.write("WARNING: no messages received and simulated time is active.\nIs /clock being published?\n")

        # while not rospy.is_shutdown() and not callback_echo.done:
        #     _sleep(0.1)


            
##########################################################################################
# COMMAND PROCESSING #####################################################################
    
def _rostopic_cmd_echo(argv):
    def expr_eval(expr):
        def eval_fn(m):
            return eval(expr)
        return eval_fn

    args = argv[2:]
    from optparse import OptionParser
    parser = OptionParser(usage="usage: %prog echo [options] /topic", prog=NAME)
    parser.add_option("-b", "--bag",
                      dest="bag", default=None,
                      help="echo messages from .bag file", metavar="BAGFILE")
    parser.add_option("-p", 
                      dest="plot", default=False,
                      action="store_true",
                      help="echo in a plotting friendly format")
    parser.add_option("--filter", 
                      dest="filter_expr", default=None,
                      metavar="FILTER-EXPRESSION",
                      help="Python expression to filter messages that are printed. Expression can use Python builtins as well as m (the message) and topic (the topic name).")
    parser.add_option("--nostr", 
                      dest="nostr", default=False,
                      action="store_true",
                      help="exclude string fields")
    parser.add_option("--noarr",
                      dest="noarr", default=False,
                      action="store_true",
                      help="exclude arrays")
    parser.add_option("-c", "--clear",
                      dest="clear", default=False,
                      action="store_true",
                      help="clear screen before printing next message")
    parser.add_option("-a", "--all",
                      dest="all_topics", default=False,
                      action="store_true",
                      help="display all message in bag, only valid with -b option")
    parser.add_option("-n", 
                      dest="msg_count", default=None, metavar="COUNT",
                      help="number of messages to echo")
    parser.add_option("--offset",
                      dest="offset_time", default=False,
                      action="store_true",
                      help="display time as offsets from current time (in seconds)")

    (options, args) = parser.parse_args(args)
    if len(args) > 1:
        parser.error("you may only specify one input topic")
    if options.all_topics and not options.bag:
        parser.error("Display all option is only valid when echoing from bag files")
    if options.offset_time and options.bag:
        parser.error("offset time option is not valid with bag files")
    if options.all_topics:
        topic = ''
    else:
        if len(args) == 0:
            parser.error("topic must be specified")        
        topic = rosgraph.names.script_resolve_name('rostopic', args[0])
        # suppressing output to keep it clean
        #if not options.plot:
        #    print "rostopic: topic is [%s]"%topic
        
    filter_fn = None
    if options.filter_expr:
        filter_fn = expr_eval(options.filter_expr)

    try:
        msg_count = int(options.msg_count) if options.msg_count else None
    except ValueError:
        parser.error("COUNT must be an integer")
        
    field_filter_fn = create_field_filter(options.nostr, options.noarr)
    callback_echo = CallbackEcho(topic, None, plot=options.plot,
                                 filter_fn=filter_fn,
                                 echo_clear=options.clear, echo_all_topics=options.all_topics,
                                 offset_time=options.offset_time, count=msg_count,
                                 field_filter_fn=field_filter_fn)
    try:
        _rostopic_echo(topic, callback_echo, bag_file=options.bag)
    except socket.error:
        sys.stderr.write("Network communication failed. Most likely failed to communicate with master.\n")

def create_field_filter(echo_nostr, echo_noarr):
    def field_filter(val):
        fields = val.__slots__
        field_types = val._slot_types
        for f, t in zip(val.__slots__, val._slot_types):
            if echo_noarr and '[' in t:
                continue
            elif echo_nostr and 'string' in t:
                continue
            yield f
    return field_filter



def cogni_topic_echo(topic, callback_echo):
    """
    Print new messages on topic to screen.
    
    :param topic: topic name, ``str``
    :param bag_file: name of bag file to echo messages from or ``None``, ``str``
    """
    # we have to init a node regardless and bag echoing can print timestamps


    _check_master()
    rospy.init_node(NAME, anonymous=True)
    msg_class, real_topic, msg_eval = get_topic_class(topic, blocking=True)
    if msg_class is None:
        # occurs on ctrl-C
        return
    callback_echo.msg_eval = msg_eval

    # extract type information for submessages
    type_information = None
    if len(topic) > len(real_topic):
        subtopic = topic[len(real_topic):]
        subtopic = subtopic.strip('/')
        if subtopic:
            fields = subtopic.split('/')
            submsg_class = msg_class
            while fields:
                field = fields[0].split('[')[0]
                del fields[0]
                index = submsg_class.__slots__.index(field)
                type_information = submsg_class._slot_types[index]
                if fields:
                    submsg_class = roslib.message.get_message_class(type_information)

    use_sim_time = rospy.get_param('/use_sim_time', False)
    sub = rospy.Subscriber(real_topic, msg_class, callback_echo.callback, {'topic': topic, 'type_information': type_information})
    _subscribers[topic] = sub
    if use_sim_time:
        # #2950: print warning if nothing received for two seconds

        timeout_t = time.time() + 2.
        while time.time() < timeout_t and \
                callback_echo.count == 0 and \
                not rospy.is_shutdown() and \
                not callback_echo.done:
            _sleep(0.1)

        if callback_echo.count == 0 and \
                not rospy.is_shutdown() and \
                not callback_echo.done:
            sys.stderr.write("WARNING: no messages received and simulated time is active.\nIs /clock being published?\n")

    # while not rospy.is_shutdown() and not callback_echo.done:
    #     _sleep(0.1)


_active_listeners = {}
_subscribers = {}

def update_topics():
	# Replace with topics from C++
	topics = cogni_topic_listener.internal_get_topics()

	# Add new topics to listeners
	for new_topic in topics:
		if new_topic not in _active_listeners:
			add_topic_listener(new_topic)
			# print("New topic added '" + new_topic + "'")

	# Remove remove topics
	for active_topic in _active_listeners.keys():
		if active_topic not in topics:
			_subscribers[active_topic].unregister()
			del _active_listeners[active_topic]
			del _subscribers[active_topic]
			# print("Topic removed '" + active_topic + "'")


def add_topic_listener(topic):
	callback_echo = CallbackEcho(topic, None)
	cogni_topic_echo(topic, callback_echo)
	_active_listeners[str(topic)] = callback_echo

def is_interruption_request():
	return cogni_topic_listener.internal_interruption_requested()

# add_topic_listener("/scan/ranges[0:1]")
# add_topic_listener("/scan/ranges[1:2]")
# add_topic_listener("/scan/ranges[2:3]")

while True:
	_sleep(1)
	update_topics()	
	if (is_interruption_request()):
		break
