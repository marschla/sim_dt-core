# dt-core

Contains Duckietown Controllers for simulator.

Usual procedure to run the simulator:
1) start the sim-wrapper 
2) start the controllernode inside this repo:
	docker run -it --rm --net=host -v /PATH_TO_CALI_FILE:/data -e SCALE=X -e NODENAME=CONTROLLER 
	duckietown/sim_dt-core:main-amd64
   Where X regulates how strong the magnitude of the disturbance is and CONTROLLER specifies the name of the Controller.
3) run dt start_gui_tools fakebot then rqt_image_view to see the simulated camerafeed

