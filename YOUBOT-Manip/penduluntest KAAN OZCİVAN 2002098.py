from coppeliasim_zmqremoteapi_client import *
import random
import numpy as np

client = RemoteAPIClient()
sim = client.require('sim')

rev1 = sim.getObject("/Revolute_joint1")
rev2 = sim.getObject("/Revolute_joint2")
endeff = sim.getObject("/Dummy")

sim.setStepping(True)
sim.startSimulation()
while 1:
    xref=1
    yref=-0.3
    l=1
    theta1=sim.getJointPosition(rev1)
    theta2=sim.getJointPosition(rev2)
    
    J=np.array([[-l*(np.sin(theta1)-np.sin(theta1+theta2)), -l*np.sin(theta1+theta2)],[l*(np.cos(theta1)+np.cos(theta1+theta2)),l*np.cos(theta1+theta2)]])
    Jinv = np.linalg.inv(J)
    
    t = sim.getSimulationTime()
    
    pos5=sim.getObjectPosition(endeff, sim.handle_world)
    x=pos5[0]
    y=pos5[1]
    altreff=[x,y]
    dr = np.array([xref - x, yref - y])
    dq = Jinv.dot(dr)
    
    theta1 += dq[0]
    theta2 += dq[1]

    sim.setJointTargetPosition(rev1, theta1)
    sim.setJointTargetPosition(rev2, theta2)

    sim.setJointTargetVelocity(rev1,0.01)
    sim.setJointTargetVelocity(rev2,0.01)
    
    s=print('x '+str(x)+ ' y= '+ str(y))
    if t >= 40:
     break
    sim.addLog(sim.verbosity_scriptinfos, s)
    sim.step()
sim.stopSimulation()

