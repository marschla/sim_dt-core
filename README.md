# dt-core

Contains Duckietown Controllers for simulator.

Usual procedure to run the simulator:
1) start the sim-wrapper found here: https://github.com/marschla/sim-wrapper

2) start the controllernode inside this repo:
```shell script
 docker run -it --rm --net=host -v /PATH_TO_CALI_FILE:/data -e SCALE=X -e NODENAME=CONTROLLER 
	duckietown/sim_dt-core:main-amd64
```
   Where X regulates how strong the magnitude of the disturbance is and CONTROLLER specifies the control architecture, see the launch files and the .sh file for the naming conventions.

3) run dt start_gui_tools fakebot then rqt_image_view to see the simulated camerafeed

4) Note: the implemented controllers can be found in my_package

Note: The simulator feeds the exact Lane Pose information the the simulated robot, thus there is no error from the pose estimation.

