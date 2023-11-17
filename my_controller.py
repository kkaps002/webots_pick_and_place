# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Demonstration of inverse kinematics using the "ikpy" Python module."""


import sys
import tempfile
try:
    import ikpy
    from ikpy.chain import Chain
except ImportError:
    sys.exit('The "ikpy" Python module is not installed. '
             'To run this sample, please upgrade "pip" and install ikpy with this command: "pip install ikpy"')

import math 
from controller import Supervisor, VacuumGripper, DistanceSensor

if ikpy.__version__[0] < '3':
    sys.exit('The "ikpy" Python module version is too old. '
             'Please upgrade "ikpy" Python module to version "3.0" or newer with this command: "pip install --upgrade ikpy"')

IKPY_MAX_ITERATIONS = 4

# Initialize the Webots Supervisor.
supervisor = Supervisor()
timeStep = int(4 * supervisor.getBasicTimeStep())

# Create the arm chain from the URDF
filename = None
with tempfile.NamedTemporaryFile(suffix='.urdf', delete=False) as file:
    filename = file.name
    file.write(supervisor.getUrdf().encode('utf-8'))

armChain = Chain.from_urdf_file(filename, active_links_mask=[False, True, True, True, True, True, True, False])
print(armChain.links)
# Initialize the arm motors and encoders.
motors = []
for link in armChain.links:
    if 'motor' in link.name:
        motor = supervisor.getDevice(link.name)
        motor.setVelocity(1.0)
        position_sensor = motor.getPositionSensor()
        position_sensor.enable(timeStep)
        motors.append(motor)
             
#---------------------------------------------------------------------------------------------------------------------------      


     
#initialize gripper
gripper = VacuumGripper("gripper")

gripper.enablePresence(100) #gripper presence sampling period
# print(gripper.getPresenceSamplingPeriod())
#print(gripper.getPresence())
#print(gripper.isOn())

#initialize sensor
ds = DistanceSensor("ds")
ds.enable(1)
#print(ds.getLookupTable())

# Get the arm nodes.
arm = supervisor.getSelf()

#---------------------------------------------------------------------------------------------------------------------------      

pickRestPosition = [1.29, -0.05, 1.4]
PICK_POSITION = [1.44, -0.048, 0.841]

BOX_GRABBED = [1.44, -0.048, 2]

# placeRestPosition_z = [1.32, -0.048, 1.5]

placeRestPosition = [-0.19, -1.575, 1.35]



box_x1 = 0.216
box_x2 = -0.19
box_x3 = -0.6



box_y1 = -1.30911
box_y2 =  -1.58
box_y3 = - 1.85

box_z = 0.5
box_z2 = 1 #z pos for place Rest positions

placePositions = [[box_x1, box_y1, box_z], [box_x1, box_y2, box_z], [box_x1,box_y3, box_z],[box_x2, box_y1, box_z], [box_x2, box_y2, box_z], [box_x2,box_y3, box_z], [box_x3,box_y1, box_z],[box_x3, box_y2, box_z], [box_x3, box_y3, box_z]]

placeRestPositions = [[box_x1, box_y1, box_z2], [box_x1, box_y2, box_z2], [box_x1,box_y3, box_z2],[box_x2, box_y1, box_z2], [box_x2, box_y2, box_z2], [box_x2,box_y3, box_z2], [box_x3,box_y1, box_z2],[box_x3, box_y2, box_z2], [box_x3, box_y3, box_z2]]

targetPosition = pickRestPosition #initialise target position

#define

counter = 0
count_start = False
index = 0

while supervisor.step(timeStep) != -1:
    if index == 9:
        break #stop execution when out of boxes
    placePosition = placePositions[index]
    placeRestPosition = placeRestPositions[index]
    
    if count_start:
        counter = counter + 1
        print(counter)
    # else:
        # count_start = True
        # counter = counter + 1
        # print("Waiting")
        # if counter == 20:
            # counter = 0
            # count_start = True
            
            
            
   
    # Get the absolute postion of the target and the arm base.
    armPosition = arm.getPosition()
    
    # Compute the position of the target relatively to the arm.
    #define positions outside of while loop and rotate between them - the definitions here are re-implemented every TIME_STEP
    # x and y axis are inverted because the arm is not aligned with the Webots global axes.
    x = targetPosition[0] - armPosition[0]
    y = targetPosition[1] - armPosition[1]  
    z = targetPosition[2] - armPosition[2]
    
    # #if ds laser is enabled check for boxes and enable gripper
    if not ds.getSamplingPeriod() == 0: 
        print(ds.getValue())
        #disable ds laser when box is detected
        if (ds.getValue()) < 6400.0 and (ds.getValue()) != 0:
             count_start = True
             print("BOX DETECTED", ds.getValue(), "targetPos:", targetPosition)
             
             ds.disable() 
             targetPosition = PICK_POSITION
             print("PickPos:", PICK_POSITION)
             print ("NewTarget:", targetPosition)
             gripper.turnOn()
             
                 
     
    #check if the gripper is attached to a box or not
    if gripper.isOn():
        
        if gripper.getPresence():
            if counter == 10:
               print("LIFT_BOX")
               targetPosition = BOX_GRABBED
               motors[5].setPosition(math.pi/2)
               
                                      
            if counter == 30:
                targetPosition = placeRestPosition
                # targetPosition = placePosition
                # targetPosition[2] = targetPosition[2] + 2
                print("PLACE_REST AT:", targetPosition)
        
            if counter == 50:
                targetPosition = placePosition
                # targetPosition[2] = targetPosition[2] - 2
                print("PLACE AT:", targetPosition)
                 
            if counter == 70:
                gripper.turnOff()
               
                
    if counter == 80:
            targetPosition[2] = 2    
            print("LIFT ARM")
            
                
    if counter == 90:
        targetPosition = pickRestPosition
        # targetPosition[2] = targetPosition[2] - 0.4
        print("WAIT FOR NEXT BOX")
        # count_start = False
            
    if counter == 110: 
        ds.enable(1)
        counter = 0
        index = index + 1
        print(index)
        count_start = False
            
            
         
                      
               
 
        

        # ds.enable(100)
        # gripper.turnOn()
       
        
    #print(placePosition)
    #-------------------------------------------------------------------------------------------------------------------------      
    # Call "ikpy" to compute the inverse kinematics of the arm.
    initial_position = [0] + [m.getPositionSensor().getValue() for m in motors] + [0]
    ikResults = armChain.inverse_kinematics([x, y, z], max_iter=IKPY_MAX_ITERATIONS, initial_position=initial_position)
     
    
    # Recalculate the inverse kinematics of the arm if necessary.
    position = armChain.forward_kinematics(ikResults)
    squared_distance = (position[0, 3] - x)**2 + (position[1, 3] - y)**2 + (position[2, 3] - z)**2
    if math.sqrt(squared_distance) > 0.03:
        ikResults = armChain.inverse_kinematics([x, y, z])
    
    # Actuate the arm motors with the IK results.
    for i in range(3):
        motors[i].setPosition(ikResults[i + 1])
        
    motors[4].setPosition(-ikResults[2] - ikResults[3] + math.pi / 2)
    motors[5].setPosition(ikResults[1] + math.pi/2)