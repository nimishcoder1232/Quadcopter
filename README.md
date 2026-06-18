# DIY-Quad-Copter
I want to make a drone that is completely autonomous, with my own flight computer. I will not use any drone control software; rather, I will make it all. The code from scratch, to learn as much as I can.

For the outpost submission, I will request S teir, rather than A teir. My reasons are following:

1. Custom Flight Control Software

The drone's flight software is entirely custom and has been developed from scratch by me. Unlike drones that rely on commercial off-the-shelf (COTS) flight control software such as Betaflight or similar systems, this project requires the development of a complete flight control stack, including sensor fusion, state estimation, and stabilization algorithms.

Developing this software has required extensive research into control systems, quaternion-based orientation estimation, complementary filters, and drone dynamics. Additionally, I anticipate multiple iterations and redesigns of the software before achieving reliable flight performance, making this a highly ambitious component of the project.

2. GPS-Based Autonomous Navigation

This drone is designed to support GPS navigation capabilities. The current codebase already contains foundational GPS-related functionality, and future development will include point-to-point autonomous navigation.

Implementing autonomous navigation significantly increases the complexity of the system, as it requires integration of GPS data with the flight controller, position estimation, waypoint tracking, and autonomous control logic.

3. Custom Ground Control System

In the PCB folder, you can see that I designed my own drone control board, rather than a COTS solution. 

CAD Screenshot: 
<img width="1377" height="738" alt="image" src="https://github.com/user-attachments/assets/313e1d1b-069d-40ca-ad3a-b42b08b07887" />

PCB Schematic: 
<img width="863" height="772" alt="image" src="https://github.com/user-attachments/assets/c743a796-15a3-42f9-8fc0-a0172a5565ed" />

PCB Routing: 
<img width="900" height="635" alt="image" src="https://github.com/user-attachments/assets/d1f0698b-d9b0-4067-9042-800967fb8c3e" />






