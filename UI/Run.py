#	Run button
def run_mode(selected, object, joints, coordinates):
	
	if selected == 1:
		object = entry_1.get()
		if object == "":
			print("no object defined")
			return
		print(f"Running object detection on {object}")
		Run.run_mode_1(object)
		#want it to open new frame of camera view 
 
	elif selected == 2:
		val = [j_entry.get() for j_entry in joint_entries]
		if "" in val:
			print("no joints defined")
			return

		try:
			joints = [float(v) for v in val]
	
		except ValueError:
			print("Invalid input")
			return

		print("Running set joint positions")
		Run.run_mode_2(joints)

 	
	elif selected == 3:
		val = [c_entry.get() for c_entry in coordinate_entries]
		if "" in val:
			print("no coordinates defined")
			return

		try:
			coordinates = [float(v) for v in val]

		except ValueError:
			print("Invalid input")
			return

		print("Running coordinate positions")
		Run.run_mode_3(coordinates)

	else:
		print("No mode selected")
		return


