#!/usr/bin/python

#from sys import *
#from os import *

dot_file = open('ICD.dot','r').read().split('\n')

data={}
graph=[]
nodes=[]
state='start'
for line in dot_file:
	line = line.strip()
	if line == '#TYPES':
		state = 'types'
		continue
	if line == '#GRAPH':
		state = 'graph'
		continue
	if state=='start':
		continue
	if state=='types':
		if len(line)==0: continue
		if "label" not in line: continue
		v,k = line.split(' [label="')
		a,t = k[:-3].split(" [")
		data[v]=(a,t)
		continue
	if state=='graph':
		if len(line)==0: continue
		if "->" not in line: continue
		f,d,t = line.split("->")
		if '{' in f:
			f = f.strip()[1:-1].split(' ')
		else:
			f = [f]
		if '{' in t:
			t = t.strip()[1:-1].split(' ')
		else:
			t = [t]
		if '{' in d:
			d = d.strip()[1:-1].split(' ')
		else:
			d = [d]
		def strip_all(x):
			return [n.strip() for n in x]
		f=strip_all(f)
		d=strip_all(d)
		t=strip_all(t)
		for n in f+t:
			if n not in nodes: nodes.append(n)
		if d not in (['event'],['diagnostic']):
			graph.append( (f,d,t) )
		continue

def CreateConf():
	res = []
	for n in nodes:
		res.append( "#"+n )
		pub = [ t for t in graph if n in t[0] ]
		sub = [ t for t in graph if n in t[2] ]
		for kk in sub:
			for k in kk[1]:
				for m in kk[0]:
					res.append( n+"_sub_"+k+": '"+m+"'" )
		res.append( "" )
		for kk in pub:
			for k in kk[1]:
				res.append( n+"_pub_"+k+": '"+data[k][0]+"'" )
		res.append( "" )
	return "\n".join(res)
		
def CreateTypes():
	res = []
	res.append( "#ifndef PARAMETERTYPES_H_" )
	res.append( "#define PARAMETERTYPES_H_" )
	res.append( '#include "ParameterTypesDefs.h"' )
	res.append( "" )
	res.append( "//PUBLICATIONS" )
	for n in nodes:
		pub = [ t for t in graph if n in t[0] ]
		for kk in pub:
			for k in kk[1]:
				res.append( "DEF_PUB( "+n+", "+k+", "+data[k][1]+" )" )
		res.append( "" )

	res.append( "//SUBSCRIPTIONS" )
	for n in nodes:
		sub = [ t for t in graph if n in t[2] ]
		for kk in sub:
			for k in kk[1]:
				for m in kk[0]:
					res.append( "DEF_SUB( "+n+", "+k+", "+m+" )" )
		res.append( "" )
	res.append( "#undef DEF_PUB" )
	res.append( "#undef DEF_SUB" )
	return "\n".join(res)
		
		
open('configuration.yaml','w').write(CreateConf())
open('include/ParameterTypes.h','w').write(CreateTypes())
