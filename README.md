# cooperative-navigation-framework
step 1: run the setUPModel_UScityblock2.m editor file
		this file will load all the data of the designed scenario into the workspace for example 
		1) actor vehicles' velocities, position, waypoints, and path at every instant 
		2) road curvature, intersections parameters, road map information
step 2: run the ego_setup_model.m file
		this file will load the data of the ego vehicle into the workspace for example
		1) waypoints, followed the path, speed at each waypoint
step 3: run the waypoint_profile.mlx 
		this file will generate an equal number of waypoints for each vehicle in relation to the simulation time 
		for example, if the simulation time is set to be 50 sec, therefore, each vehicle path is divided by 50 sets of waypoints
		depending on their velocity and position.
step 4: run the velocity_profile.mlx
		this file will generate velocity points according to the waypoints of each vehicle.
	
Step5: run the CAAlorithmforAIMV2VSTLcooperation.mlx (or any other scenario file)
		This will execute the cooperative collision avoidance algorithm and save the results in the workspace.

		Now you have all the variables in the workspace to execute the Simulink model 
Step 6: run the Simulink models according to the scenario e.g (MPC_intersection_V2VandAIM.slx) to generate the vehicle dynamics and control algorithm responses    
