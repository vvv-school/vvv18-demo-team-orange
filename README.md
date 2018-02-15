# VVV18 ORANGE TEAM

- 📚 [**Project description**](https://github.com/vvv-school/vvv18/wiki/Team-contest)
- 👋 [**Meet the team**](https://github.com/orgs/vvv-school/teams/vvv18-team-orange/members)
- :feet: [**Project management**](https://github.com/vvv-school/vvv18-demo-team-orange/projects/1)
- 🗣 [**Issues**](https://github.com/vvv-school/vvv18-demo-team-orange/issues)

# Documentation

Ciao a tutti and welcome to the official documentation of the **VVV18 Orange Team** repository for the final project of this amazing school!

Let's start with a brief overview of the problem itself and then move on how we decided to tackle it.

## Overview

So, we were asked to make iCub able to:
 - look at two objects shown by the human in front of him;
 - classify them correctly and point at the one asked by the human;
 - wait for an acknowledgement from the human to understand if he succeeded or not and behave consequently, showing happiness or sadness.

The approach we decided to follow has been inspired by the YARP modularity, leading us to a State-Machine architecture. 
![application](misc/framework.png)

The central role is played by a **manager application**, which is in charge of communicating with the modules available using a rpc protocol and sending commands (with data attached) or even just triggering signals. The manager is also, of course, in charge of defining the temporal sequence of actions, waiting for an ack after each single operation. The temporal sequence can be defined as follows:

//here we put obvs the single modules

The other modules are divided in according with the topic subdivision followed during the school and provide the function implementations needed from the manager to carry out the demo. Only the Kinematics module implements also the Gaze control, in order to avoid "empty modules" (that are modules which just one function or few lines of code). This finds support also in the YARP implementation of the two controllers, which share the same basic idea of control and implementation through PolyDrivers, interfaces and so on. Let's have a look at each module in details:

### Vision


### Dynamics
As we learnt from the *Robot Dynamics* lecture, iCub mounts on the body sensors able to perceive generalized forces applied to the end effector. This has been exploited starting from the consideration that having the robot hand in *high five* or *low five* position means that our end-effector frame has the axes almost collinear with the root frame (even if the direction is different or the y_ee is collinear with the x_root). This means that, once we established the gestures we intend to do to confirm or reject the classification, we just need to read the force applied along the axis of interest to discriminate the two cases. The magnitude of the force, if higher than a certain threshold, will tell us that a contact happened; the direction will tell us if it's a positive or negative ack.
To do so, it is mandatory to reset the sensor once we reach the high/low five hand configuration. In this way, we will read always almost 0 and a higher value only in case of contact! Useful instructions have been:

`$ yarp rpc /wholeBodyDynamics/rpc:i`

to reset the sensors to 0, giving this value as input to the rpc port;

`$ yarp read ... /wholeBodyDynamics/right_arm/cartesianEndEffectorWrench:o`

to read forces and wrenches felt by the sensor and applied to the end-effector;

`$ yarpscope --remote "/wholeBodyDynamics/right_arm/cartesianEndEffectorWrench:o`

to open a yarp scope and link it to the sensor readings, in order to understand the direction of the applied forces and to tune the threshold to detect the contact.




#### Dependencies
- [robotology/segmentation](https://github.com/robotology/segmentation)


