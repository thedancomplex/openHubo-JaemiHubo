#!/usr/bin/env python

from openravepy import *
from numpy import *
import time
import sys
import curses
import traceback
import ctypes, ctypes.util
from ctypes import *

HUBO_JOINT_COUNT = 50

libach = ctypes.util.find_library('ach')
ach = ctypes.CDLL(libach)

hubo_ref_chan = pointer(c_long(0))

ach_open = ach.ach_open
ach_get = ach.ach_get

hubo_ref_chan = pointer(c_long(0))
r = ach_open( hubo_ref_chan, c_char_p('hubo-ref'), c_int(0))
print "Ach Ref Chan Open 0 if ok = ", r


class hubo_ref_type(Structure):
    _fields_=[("ref_array", c_double * HUBO_JOINT_COUNT)]

def makeGlobals():
    global hubo_ref_chan
    return 0


def run():

    global hubo_ref_chan
    #-- Read the name of the xml file passed as an argument
    #-- or use the default name
    try:
        file_env = sys.argv[1]
    except IndexError:
        file_env = 'jaemiHubo.robot.xml'

    env = Environment()
    
    #Viewer does not seem to consume much CPU time
    env.SetViewer('qtcoin')
    env.SetDebugLevel(3)
    env.Load(file_env)

    #-- Set the robot controller and start the simulation
    with env:
        robot = env.GetRobots()[0]

    time.sleep(2)

    #setACHReferenceDirect(robot,{'LSR':50,'RSR':-50,'REP':-20,'LEP':-20})
    setACHReferenceDirect(robot,{'LSR':1.0,'RSR':-1.0,'REP':-1.0,'LEP':-1.0})
    
    starttime=time.time()
    itrs=10000;
    #tried this also with locked, environment, no change in CPU time consumed
    for i in range(itrs):
        #Uncomment to see timing with ACH packet parser
        #setACHReferenceDirect(robot,readACHPacket('rad'))
        #uncomment to see timing of only the OpenRAVE SetDOF function
        setACHReferenceDirect(robot,{'RSP':1.2345})
	print i
        #time.sleep(.1)
    endtime=time.time()
    print "Elapsed time for {} commands is {}".format(itrs,endtime-starttime)
    print "Average cmd time is {}".format((endtime-starttime)/itrs)
    return 0 

def setACHReferenceDirect(robot,refPos):
    ind=zeros(len(refPos))
    k=0
    for x in refPos.keys():
        #print "setting joint ",x,"to angle ",refPos[x]
        ind[k]=robot.GetJointIndex(x)
        k+=1
    #This would be even faster if a full complement of angles were included
    robot.SetDOFValues(refPos.values(),ind)

def readACHPacket(units):
    #Eventually use this function to access the ACH channel and get the most
    #recent set point. For now, make up random values to send
    deg2rad=numpy.pi/180
    setPoint={'LSP':0.0,'LSR':80.0,'LSY':0.0,'LEP':-50,'LWY':0.0,'LWP':0.0,'RSP':0.0,'RSR':-80.0,'RSY':0.0,'REP':-50,'RWY':0.0,'RWP':0.0}
    for x in setPoint.keys():
        setPoint[x]+=numpy.random.rand()*20.0-10.0
        if units=='rad':
            setPoint[x]=setPoint[x]*deg2rad

    return setPoint



if __name__=='__main__':
    makeGlobals()
    run()
    print libach
    print ach_open
    print ach_get
    print "hubo_ref_chan = ", hubo_ref_chan
    raw_input("Press Enter to continue...")
    sys.exit('Exiting')
