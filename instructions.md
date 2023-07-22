# How to setup the Unitree Go1: HSL Edition

## VM
1. create ubuntu vm (iso in drive)
2. network setup - 2 network interfaces:
   - vmnet0 (bridged to intel dual band wireless) - lets you connect to dog, confirm that this is actually connected to wifi using edit > virtual network editor > change settings button at bottom right
   - NAT - connect to internet

3. make sure windows is actually connected to dog wifi, needs to be reconnected if dog is rebooted

4. in ubuntu, sudo apt install: (these can all be installed in one line like "sudo apt install [package1] [package2] [package3] etc")
    - net-tools
    - git
    - ssh (if you want to ssh into the vm)
    - cmake
    - g++
    - libboost-all-dev (this one might take a while)

5. run ifconfig to ensure there are 2 network devices
6. make sure you can successfully ping 8.8.8.8 (or some other internet IP) and 192.168.123.161 (dog pi)

7. git clone https://github.com/lyphix/unitree_matlab.git
8. cd into unitree_matlab
9. edit example/udp_link.cpp line 33 (if using vi, use ":set number" to show line numbers)
10. make sure the ip is "192.168.123.161" - dog pi IP
11. this might be set to 127.0.0.1 by default
12. cd back into unitree_matlab if you aren't already there
13. mkdir build
14. cd build
15. cmake ..
16. make

17. if you make changes to any of the cpp files, you will need to run make again, but cmake does not need to be repeated unless you edit/change your Makefile, CMakeFiles, cmake_install.cmake, or CMakeCache.txt

18. from the build directory, run ./example_walk to ensure the vm is connected to the dog
19. you'll be prompted to click Enter to start the program
20. if you don't want to run the full walking sequence, click ctrl+C

## MOTIVE
1. open motive on windows
2. in the quick start menu, choose "Open Camera Calibration"
3. in the File Explorer window that appears, go to D:\OptitrackCalibrationFiles and open the newest calibration (CAL) file
4. you should be returned to the Motive window with a black grid plane, and if you zoom out, 12 cameras
5. place the dog at the center of the duct tape circle (there should be a duct tape X) and face it away from the computers (west)
6. ensure the legs are not underneath the dog, are folded, and with the feet facing the same direction as the dog itself
7. in Motive, find the 3 dots that represent the 3 reflective balls on top of the dog. click and drag to select them (and nothing else), then right click on them and select Rigid Body > Create From Selected Markers
you should now see a colored triangle with 3 colored balls at the edges, and a yellow ball at the center
8. in the left sidebar, under Assets (Live), check the Rigid Body number that corresponds to the dog. this should be Rigid Body 1 (if not, please remember the number as it will be important later)
9. to test, move the dog around and ensure that the rigid body in Motive moves along with it

10. from the menu at the top of the Motive window, go to View > Data Streaming
11. a new set of options should open at the right side of Motive
12. select the IP address next to Local Interface (under Network Interface) and change it to Local Loopback
13. check the box to turn on Broadcast Frame Data in the OptiTrack Streaming Engine section

## MATLAB
1. open matlab on windows
2. for these instructions, we will use Xi Luo's code from 2023, located at D:\xluo45\02MATLAB\High_Level_Control. if you wish to make edits or do your own experiments, please duplicate this folder to another location and use that
3. in matlab, open the High_Level_Control folder (or whatever directory you're working out of)
4. open Main.m
5. on line 7, change the Robot_Dog_IP variable:
   - in ubuntu, run ifconfig. look at the network interface ens33, and find the IP address next to inet. it should resemble 192.168.254.133
   - set Robot_Dog_IP to this IP address (as a string, in quotes)
6. on line 49, ensure Control_Mode is set to 2. this will follow a series of waypoints (which are placed around the duct tape circle)
7. on line 55, change yaw_set as desired. if yaw_set is -1, the dog will simply "strafe" around the circle, facing the initial orientation the entire time. if yaw_set is -2, the dog will walk around the circle facing the direction it is moving, like a regular dog (technically)
8. on line 131, set Dog_ID to the Rigid Body number of the dog from Motive. by default, this should be 1

## RUN
1. in the vm, cd into the build directory, then run ./udp_link. when prompted, click Enter
2. in matlab, run Main.m (the green play button at the top of the screen) WARNING: the robot will now begin to move
3. matlab should open a new window showing the circle of target points, the dog's current position and direction, and (highlighted in red) the current point the dog is moving toward. When the dog reaches the red waypoint, the next waypoint on the circle will instead be highlighted so the dog can continue moving around the circle
4. as the dog follows the waypoints, a red line will be drawn to show the dog's path
5. when the dog has made its way around the circle, matlab will open 4 new plot windows to display the dog's x over time, z over time, yaw over time, path, and speed over time

## TROUBLESHOOTING
there are 4 points of failure in this setup: motive to matlab, matlab to ubuntu, ubuntu to the dog, and the dog's control itself

### motive to matlab
- in motive, ensure the 3 points of the dog are correctly created as a rigid body
- check that the rigid body number corresponding to the dog in motive is the same as the Dog_ID on line 131 of Main.m
- in motive, check that the broadcast frame data checkbox is checked (specifically the one under OptiTrack Streaming Engine)
- in motive, check that Local Interface under Network Interface under OptiTrack Streaming Engine is set to Local Loopback

### matlab to ubuntu
- check that the Robot_Dog_IP on line 7 of Main.m is set to the inet address of ubuntu's network interface ens37 (run ifconfig in ubuntu to find this, make sure you can successfully ping this IP from windows)
  - the IP address should resemble 192.168.254.X
  - if it does not, then also change the IP address on line 5 of Robot_Dog.m to match the IPv4 address of Ethernet adapter VMware Network Adapter VMnet8 (which can be found by running ipconfig on windows)
  - this should resemble X.X.X.1, where the first 3 numbers match the first 3 numbers of the ubuntu IP that was used
- ubuntu's firewall may block port 1145, which is used to communicate between Main.m and udp_link
  - to unblock this, run sudo ufw allow 1145/udp on ubuntu
- the ControlCommand being sent to ubuntu by matlab should be of type 'single'. to do this, use the command "Control_Command = zeros(1,11,'single');" in matlab when instantiating Control_Command
- use udp_link's output to see what data it is receiving from matlab

### ubuntu to dog
- check that windows is connected to wifi
- check that the two network adapters are set up correctly (VMnet0 and NAT)
- check that you can successfully ping 192.168.123.161 from ubuntu
- check that the IP on line 33 of udp_link.cpp is set to 192.168.123.161
  - run make from the build directory to ensure the udp_link executable is up to date
- check that example_walk works, to help isolate your issue

### dog control
- check that yaw_set (Main.m line 55) is set as desired
- check that Control_Mode (Main.m line 49) is set as desired
- if the dog seems to be going haywire, check that its direction (depicted by an arrow in the Matlab plot) matches the direction it is physically facing
  - the bottom of the plot window is the side of the wall with the computers (east)
  - if this direction is incorrect, turn the dog to face the wall away from the computers (west). in motive, select Rigid Body 1 in the left sidebar under Assets (Live), then right-click it and select Settings. in the new Rigid Bodies pane, select the Orientation tab and click Reset To Current Orientation
- the dog's yaw value oscillates as it moves around the circle, so its path may not perfectly match the circle. unless it is going entirely in the wrong direction, give it a few seconds to see if it will self-correct
  - if this becomes too much of a problem, it may be helpful to change yaw_set (Main.m line 55) to -1, and see if the dog can navigate the circle without yaw control
- if it seems like the dog is trying but wildly over- or undershooting the waypoints, your PID values may be off. in this case, you will need to do a lot more trial and error with the PID constants (Main.m lines 17 to 30)