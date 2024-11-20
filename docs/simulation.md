## numerical simulation
-----------------------
- use the --pinocchio-only argument to numerically integrate the dynamics
  without sending any commands
- if you want to simulate external disturbances, put them into the kinematics/dynamics equations (create a new control loop with this)



## using ur_sim (BROKEN ATM)
-----------------------------
1. install docker (sudo apt-get install docker)
2. have a look at docker documentation to have an idea of what it is and for any additional information 
   as it's very well made
3. navigate to the the simulation folder of this repo
4. run "docker build -t myursim -f Dockerfile ." to crete the image
5. to run the image, run "docker run --rm -it --net=host myursim" . the --rm flag removes the previous container, -it attaches the terminal to the containter, and --net=host makes the image share the localhost connection with your OS.
6. go to http://127.0.1.1:6080/vnc.html?host=127.0.1.1&port=6080, go to the installation tab and change the settings (IP addresses) of the external_control urcap in the URCaps tab. change the Host IP to 127.0.0.1 
7. go to the program tab, and put external control as a part of the program (the only part after main)
8. now run your program as if it was running on real hardware use the --simulation flag, but NOTE: that there is no gripper in the simulation
