import customtkinter as ctk

#from Run import MoveARM	#run button file
#from camera_window import CameraWindow

customtkinter.set_appearance_mode("dark")  # Modes: "System" (standard), "Dark", "Light"
customtkinter.set_default_color_theme("blue")  # Themes: "blue" (standard), "green", "dark-blue"

#	reset so no repeats
camera_window = None

app = customtkinter.CTk()
app.geometry("400x880")
app.eval('tk::PlaceWindow . center')
app.title("ARM user control")
#----------------------------
#	main frame
main_frame = customtkinter.CTkScrollableFrame(app)
main_frame.pack(fill = "both", expand = "True", pady=10, padx=10)

frame_1 = customtkinter.CTkFrame(main_frame)
frame_1.pack(pady=20, padx=60, fill="x")

#	top label
label_1 = customtkinter.CTkLabel(master=frame_1, text = "ARM mode selection")
label_1.pack(pady=10)

#	text box defining modes
text_1 = customtkinter.CTkTextbox(master=frame_1, width=260, height=70)
text_1.pack(pady=10)
text_1.insert("0.0", 
	"mode 1: Object detection with NanoOWL\n"
	"mode 2: Setting joint variables\n"
	"mode 3: Set points in space\n"
)
text_1.configure(state="disabled")	# text is read only
#----------------------------
#	cant have inputs on same frame
mode = customtkinter.IntVar(value=0)

#	radiobutton frame 
radframe = customtkinter.CTkFrame(frame_1)
radframe.pack(fill = "x")

#	input frame 
inframe = customtkinter.CTkFrame(frame_1)
inframe.pack(fill = "x")
#----------------------------
#	mode inputs
#----------------------------
#	mode 1 
entry_1 = customtkinter.CTkEntry(inframe, placeholder_text="Object")
entry_1.grid(row=1, column=0, padx = 60,pady=5)
entry_1.grid_remove()	#	hide initially

#----------------------------
#	mode 2
joint_entries = []
for i in range (5):
	j_entry = customtkinter.CTkEntry(inframe, placeholder_text= f"Joint {i + 1} (deg)")
	j_entry.grid(row=i, column=0, padx = 60,pady=5)
	j_entry.grid_remove()#	hide initially
	joint_entries.append(j_entry)
#----------------------------
#	mode 3
coordinate_entries = []
for i, lbl in enumerate(("X", "Y","Z")):
	c_entry = customtkinter.CTkEntry(inframe, placeholder_text= f"{lbl}")
	c_entry.grid(row=i, column=0, padx = 60,pady=5)
	c_entry.grid_remove()#	hide initially
	coordinate_entries.append(c_entry)

#----------------------------
def show_inputs():
	global camera_window
	selected = mode.get()
	entry_1.grid_remove()
	for j_entry in joint_entries:
		j_entry.grid_remove()
	for c_entry in coordinate_entries:
		c_entry.grid_remove()

	if selected == 1:
		entry_1.grid()
		#if camera_window is None or not camera_window.winfo_exists():
		#	camera_window = CameraWindow(app)
		#else:
		#	camera_window.focus()
	#else:
	#	if camera_window is None or not camera_window.winfo_exists():
	#		camera_window.close()
	elif selected == 2:
		for j_entry in joint_entries:
			j_entry.grid()
	elif selected == 3:
		for c_entry in coordinate_entries:
			c_entry.grid()
#----------------------------
#	mode selection buttons
radiobutton_1 = customtkinter.CTkRadioButton(radframe, variable=mode, text = "mode 1", value=1, command = show_inputs)
radiobutton_1.pack(pady = 10)

radiobutton_2 = customtkinter.CTkRadioButton(radframe, variable=mode, text = "mode 2", value=2,  command = show_inputs)
radiobutton_2.pack(pady = 10)

radiobutton_3 = customtkinter.CTkRadioButton(radframe,variable=mode, text = "mode 3", value=3,  command = show_inputs)
radiobutton_3.pack(pady = 10)


#----------------------------
#	bottom frame
addframe = customtkinter.CTkFrame(frame_1)
addframe.pack(fill = "x", pady = 10)

tabsframe = customtkinter.CTkFrame(frame_1)
tabsframe.pack(fill = "x", pady = 10)
tabview_1 = customtkinter.CTkTabview(tabsframe, width=300)
tabview_1.pack(pady=10)

tabview_1.add("joint positions")
tabview_1.add("coordinates")

joint_frame = customtkinter.CTkScrollableFrame(tabview_1.tab("joint positions"), height=180)
joint_frame.pack(fill="both", expand=True)
 
coordinate_frame = customtkinter.CTkScrollableFrame(tabview_1.tab("coordinates"), height=180)
coordinate_frame.pack(fill="both", expand=True)
#----------------------------
#	validation button press
def validate(values):
	try:
		return all(v != "" and float(v) for v in values)
	except ValueError:
		return False
#----------------------------
#	add values to tab
joint_count = 0
coordinate_count = 0

def add_values():
	# add widgets on tabs
	global joint_count, coordinate_count
	selected = mode.get()

	if selected == 2:
		vals = [j_entry.get() for j_entry in joint_entries]
		if not validate(vals):
			return
		joint_count += 1
		customtkinter.CTkLabel(joint_frame, text=f"{joint_count}: J1={vals[0]}, J2={vals[1]}, J3={vals[2]}, J4={vals[3]}, J5={vals[4]}").pack(anchor="w", pady=2)
 

	elif selected == 3:
		vals = [c_entry.get() for c_entry in coordinate_entries]
		if not validate(vals):
			return
		coordinate_count += 1
		customtkinter.CTkLabel(coordinate_frame, text=f"{coordinate_count}: X={vals[0]}, Y={vals[1]}, Z={vals[2]}").pack(anchor="w",pady=2)

#----------------------------
#	final UI commands

button_add = customtkinter.CTkButton(addframe, text = "Add point", command = add_values)
button_add.pack(pady = 6)
#----------------------------
#	open and close gripper button

def open_gripper():
	print("Opening gripper")

def close_gripper():
	print("Closinging gripper")

button_open = customtkinter.CTkButton(master=addframe ,text = "Open Gripper", command=open_gripper)
button_open.pack(pady=6)

button_close = customtkinter.CTkButton(master=addframe ,text = "Close Gripper", command=close_gripper)
button_close.pack(pady=6)
#----------------------------
def save_file():
	input_text = entry.get()
	with open ("output.txt", "w") as file 
		file.write(input_text)
	print("values saved to file")
	
# we have j_entry c_entry and object entry(i cant remember the variable name)
	#when run is clicked this needs to run, then need code to communicate the new file to the jetson
#----------------------------

def run():
	selected = mode.get()
 
	if selected == 1:
                object = entry_1.get()
                if object == "":

                        print("no object")
                        return
                print("running object detection")
                Run.run_mode_1(object)
 
	elif selected == 2:
		val = [j_entry.get() for j_entry in joint_entries]
		if "" in val:
                        print("No values listed")
                        return
		try:
			joints = [float(v) for v in val]
		except ValueError:
			print("Invalid input")
			return
		print("running set joint positions ")
		Run.run_mod_2()
	elif selected == 3:
		val = [c_entry.get() for c_entry in coordinate_entries]
		if "" in val:
			print("No values listed")
			return
		try:
			coordinates = [float(v) for v in val]
		except ValueError:
			print("Invalid input")
			return
		print("running coordinate values")
		Run.run_mod_3()

	else:
		print("invalid or no selection")


button_run = customtkinter.CTkButton(master=main_frame ,text = "Run", command= run)
button_run.pack(pady=6)

def close():
	global camera_window
	if camera_window is not None and camera_window.winfo_exists():
		camera_window.close()
	app.destroy()
 


button_exit = customtkinter.CTkButton(master=main_frame ,text = "Exit", command= close)
button_exit.pack(pady=6)
#----------------------------

app.mainloop()
