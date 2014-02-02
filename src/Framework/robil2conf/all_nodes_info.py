#!/usr/bin/python


import os
import sys

def exe_lines(n):
	return [x.strip() for x in os.popen(n).readlines() ]
def exe_text(n):
	return os.popen(n).read()
def add_tab(tab, lines):
	return '\n'.join([ tab+line for line in lines.split('\n') ])
	
def node_name_toupper(node): return node[1:-len(node)+node.find('_node')].upper()
def extract_topic_name(line): return line[line.find('/'):-len(line)+line.find(' [')]
def extract_topic_type(line): return line[line.find('[')+1:-1]


nodes = exe_lines('rosnode list')
types = []
types_rec = {}
pub = {}
sub = {}
pub_rev = {}
sub_rev = {}

for node in nodes:
	info = exe_lines('rosnode info '+node)
	curr = None
	pub[node]=[]
	sub[node]=[]
	for line in info:
		if curr == None:
			if line.startswith('Publications:') : curr = pub
			if line.startswith('Subscriptions:'): curr = sub
		else:
			if line == "":
				curr = None
			elif '/rosout' not in line:
				curr[node].append( line[2:] )
				typ = extract_topic_type(line)#line[line.find('[')+1:-1]
				if typ not in types :
					types.append( typ )
					types_rec[typ] = exe_text('rosmsg show '+typ)
				name = extract_topic_name(line)#line[2:-len(line)+line.find(' [')]
				if name not in pub_rev :
					pub_rev[name] = []
					sub_rev[name] = []
				if curr == pub:
					pub_rev[name].append(node)
				elif curr == sub:
					sub_rev[name].append(node)

print "Components relationship:\n\n"
for node in nodes:
	print node_name_toupper(node)
	print '   Publishing:'
	for topic in pub[node]:
		name = extract_topic_name(topic)
		print '      '+topic+' >> '+', '.join([node_name_toupper(x) for x in sub_rev[name]])
	print '   Subscriptions:'
	for topic in sub[node]:
		name = extract_topic_name(topic)
		print '      '+topic+' << '+', '.join([node_name_toupper(x) for x in pub_rev[name]])

print "\n\nTypes descriptions:\n\n"
for t in types:
	print t
	print add_tab('   ',types_rec[t])
	
	
	
	
	
	
	
	
	