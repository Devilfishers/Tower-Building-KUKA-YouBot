from coppeliasim_zmqremoteapi_client import *
import random
import numpy as np

client = RemoteAPIClient()
sim = client.require('sim')

youbot = sim.getObject("/youBot")
nCube = 5

colorlist = np.zeros((nCube, 1))
cubeHand = colorlist.tolist()

cubeHand[0] = sim.getObject("/Rectangle13[0]")
cubeHand[1] = sim.getObject("/Rectangle13[1]")
cubeHand[2] = sim.getObject("/Rectangle13[2]")
cubeHand[3] = sim.getObject("/Rectangle13[3]")
cubeHand[4] = sim.getObject("/Rectangle13[4]")

j0 = sim.getObject("/youBotArmJoint0")
j1 = sim.getObject("/youBotArmJoint1")
j2 = sim.getObject("/youBotArmJoint2")
j3 = sim.getObject("/youBotArmJoint3")
j4 = sim.getObject("/youBotArmJoint4")
endeff = sim.getObject("/endeff")

youbotInitPos = [0,0,+0.09575]

sim.setJointPosition(j0, 0.38)
sim.setJointPosition(j1, 0.69)
sim.setJointPosition(j2, 0.96)
sim.setJointPosition(j3, 1.4)
sim.setJointPosition(j4, 0)

sim.setObjectPosition(youbot, youbotInitPos, sim.handle_world)

sim.setObjectColor(cubeHand[0], 0, sim.colorcomponent_ambient_diffuse, (0,0,0))
sim.setObjectColor(cubeHand[1], 0, sim.colorcomponent_ambient_diffuse, (1,0,0))
sim.setObjectColor(cubeHand[2], 0, sim.colorcomponent_ambient_diffuse, (0,1,0))
sim.setObjectColor(cubeHand[3], 0, sim.colorcomponent_ambient_diffuse, (0,0,1))
sim.setObjectColor(cubeHand[4], 0, sim.colorcomponent_ambient_diffuse, (1,1,1))

xMin = -0.1215
xMax = 0.1215
yMin = -0.225
yMax = -0.1
zMin = 0.1733
zMax = 0.1733

for n in range(nCube):
    
    xPos = xMin + (xMax - xMin) * random.uniform(0, 1)
    yPos = yMin + (yMax - yMin) * random.uniform(0, 1)
    zPos = zMin
    gamma = -np.pi + (np.pi - (-np.pi)) * random.uniform(0, 1)
    sim.setObjectOrientation(cubeHand[n],[0, 0, gamma], sim.handle_world)
    sim.setObjectPosition(cubeHand[n], [xPos, yPos, zPos], sim.handle_world)
    
    pos0=sim.getObjectPosition(cubeHand[0], sim.handle_world)
    ori0= sim.getObjectOrientation(cubeHand[0], sim.handle_world)
    
    pos1=sim.getObjectPosition(cubeHand[1], sim.handle_world)
    ori1= sim.getObjectOrientation(cubeHand[1], sim.handle_world)

    pos2=sim.getObjectPosition(cubeHand[2], sim.handle_world)
    ori2= sim.getObjectOrientation(cubeHand[2], sim.handle_world)

    pos3=sim.getObjectPosition(cubeHand[3], sim.handle_world)
    ori3= sim.getObjectOrientation(cubeHand[3], sim.handle_world)

    pos4=sim.getObjectPosition(cubeHand[4], sim.handle_world)
    ori4= sim.getObjectOrientation(cubeHand[4], sim.handle_world)

#sim.setObjectPosition(rev1, [0.32798,-0.175,0.1941], sim.handle_parent)
#sim.setObjectPosition(rev2, [0.26346,-0.175,0.3498], sim.handle_parent)
    
sim.setStepping(True)
sim.startSimulation()


#sim.setObjectPosition(rev1, [0.32798,-0.175,0.1941], sim.handle_world)
#sim.setObjectPosition(rev2, [0.26346,-0.175,0.3498], sim.handle_world)

while 1:
    scaling_factor = 0.001

    theta1=sim.getJointPosition(j0)
    theta2=sim.getJointPosition(j1)
    theta3=sim.getJointPosition(j2)
    theta4=sim.getJointPosition(j3)
    theta5=sim.getJointPosition(j4)

    data = [
    [0.9287, 0.9287, 0.9287, 0.3694, 0, 0], 
    [0, 0.3709, 0.3709, 0.3709, -0.9248, 0],
    [1.0000, 0, 0, 0, 0.0915, 0],
    [0.1662, -0.0912, -0.1104, -0.0604, 0.8070, 0],
    [0, 0.2284, 0.2764, 0.1513, 0.3162, 0],
    [0, -0.1873, -0.0412, -0.0363, -0.0620, 0]
    ]

   
    J= np.array(data)
    #epsilon = 1e-6
    Jinv = np.linalg.pinv(J)
    
    t = sim.getSimulationTime()
    
    xref0=0.10756
    yref0=-0.10555
    zref0=0.1733

    xref1=pos1[0]
    yref1=pos1[1]
    zref1=pos1[2]

    xref2=pos3[0]
    yref2=pos2[1]
    zref2=pos2[2]

    xref3=pos3[0]
    yref3=pos3[1]
    zref3=pos3[2]
    
    xref4=pos4[0]
    yref4=pos4[1]
    zref4=pos4[2]
    
    #mainreff = [xref,zref]
    pos5=sim.getObjectPosition(endeff, sim.handle_world)
    x=pos5[0]
    y=pos5[1]
    z=pos5[2]
    #altreff=[x,z]
    dr = np.array([xref0-x,yref0 - y, zref0 - z,0,0,0]) #position and orientation change for a specific point
    dr = dr.reshape((6, 1))
    dq = Jinv.dot(dr)
    
    
    # Update joint positions based on velocities (you might need to adjust the scaling factor)
    #nv1=sim.setJointTargetVelocity(rev1, dq[0] * scaling_factor)
    #nv2=sim.setJointTargetVelocity(rev2, dq[1] * scaling_factor)

    
    #s=print('Theta1= '+str(theta1)+ ' Theta2= '+ str(theta2)+ ' Joint velocities: '+ str(nv1)+' ' +str(nv2))


    theta1 += dq[0]
    theta2 += dq[1]
    theta3 += dq[2]
    theta4 += dq[3]
    theta5 += dq[4]
   
    sim.setJointTargetPosition(j0, float(theta1.item()))
    sim.setJointTargetPosition(j1, float(theta2.item()))
    sim.setJointTargetPosition(j2, float(theta3.item()))
    sim.setJointTargetPosition(j3, float(theta4.item()))
    sim.setJointTargetPosition(j4, float(theta5.item()))
    
    if t >= 30:

     break
    

    #s = print('Simulation time: %.2f [s] (simulation running synchronously to client, i.e. stepping)', t)
    #s=print(str(ori0)+str(pos0)+'\n')
    s = print("Theta1:", theta1, " Theta2:", theta2, " Theta3:", theta3, "Theta4:", theta4, "Theta5:", theta5) 
    #a=print("Theta2:", theta2)
    #s=print(pos0)   
    sim.addLog(sim.verbosity_scriptinfos, s)
    #sim.addLog(sim.verbosity_scriptinfos, a)
    sim.step()


sim.stopSimulation()



