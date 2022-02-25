#!/usr/bin/python

import sys

# print 'Number of arguments:', len(sys.argv), 'arguments.'
# print 'Argument List:', str(sys.argv)

input_file = sys.argv[1]
output_file = sys.argv[2]

# print 'Input file = ', input_file
# print 'Output file = ', output_file

# Read file
f = open(input_file, 'r')
template_file = f.read()

replacements = {}
# Read replacements
for i in xrange(3, len(sys.argv) - 1, 2):
    name = sys.argv[i]
    value = sys.argv[i + 1]

    replacements[name] = value

# Replace
for k,v in replacements.iteritems():
    template_file = template_file.replace(k, v)

# Write file
f_write = open(output_file, 'w')
f_write.write(template_file)
f_write.close()

# print template_file