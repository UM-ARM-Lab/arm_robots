#!/usr/bin/env python

import sys
import argparse

from urdf_parser_py.urdf import URDF
'''
parser = argparse.ArgumentParser(usage='Load an URDF file')
parser.add_argument('file', type=argparse.FileType('r'), nargs='?',
                    default=None, help='File to load. Use - for stdin')
parser.add_argument('-o', '--output', type=argparse.FileType('w'),
                    default=None, help='Dump file to XML')
args = parser.parse_args()

if args.file is None:
    robot = URDF.from_parameter_server()
else:
    robot = URDF.from_xml_string(args.file.read())
'''
robot = URDF.from_parameter_server()
#print(robot.links[4])
num_links = len(robot.links)
tot_m = 0
with open("data.txt", 'w') as f:
    for i in range(num_links):
        if hasattr(robot.links[i], 'inertial') and hasattr(robot.links[i].inertial, 'mass'):
            f.write(robot.links[i].name + "\n")
            f.write("mass: " + str(robot.links[i].inertial.mass) + '\n')
            print(robot.links[i].name)
            print("mass", robot.links[i].inertial.mass)
            print(robot.links[i].inertial.origin.xyz)
            tot_m += robot.links[i].inertial.mass
    f.write("total mass: "+str(tot_m))
    print(tot_m)
#print(robot)
'''
if args.output is not None:
    args.output.write(robot.to_xml_string())
'''