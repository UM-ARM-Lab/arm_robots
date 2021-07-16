#! /usr/bin/env python

from arm_robots.panda import Panda

if __name__ == '__main__':

    panda = Panda()
    panda.connect()

    # TODO: Reference med_motion.py for examples.