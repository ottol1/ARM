import customtkinter
from Run import run_mode	#run button file

customtkinter.set_appearance_mode("dark")  # Modes: "System" (standard), "Dark", "Light"
customtkinter.set_default_color_theme("blue")  # Themes: "blue" (standard), "green", "dark-blue"

app = customtkinter.CTk()
app.geometry("400x860")
app.eval('tk::PlaceWindow . center')
app.title("ARM user control")
#----------------------------
#	main frame
frame_1 = customtkinter.CTkFrame(app)
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
#	validation button press
def validate(values):
	try:
		return all(v != "" and float(v) for v in values)
	except ValueError:
		return False
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
	entry_1.grid_remove()

	for j_entry in joint_entries:
		j_entry.grid_remove()

	for c_entry in coordinate_entries:
		c_entry.grid_remove()

	if mode.get() == 1:
		entry_1.grid()
	elif mode.get() == 2:
		for j_entry in joint_entries:
			j_entry.grid()
	elif mode.get() == 3:
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


#	add values to tab
joint_count = 0
coordinate_count = 0

def add_values():
	# add widgets on tabs
	global joint_count, coordinate_count
 	
	if mode.get() == 2:
		vals = [j_entry.get() for j_entry in joint_entries]
		if not validate(vals):
			return
		joint_count += 1
		customtkinter.CTkLabel(joint_frame, text=f"{joint_count}: J1={vals[0]}, J2={vals[1]}, J3={vals[2]}, J4={vals[3]}, J5={vals[4]}").pack(anchor="w", pady=2)
 

	if mode.get() == 3:
		vals = [c_entry.get() for c_entry in coordinate_entries]
		if not validate(vals):
			return
		coordinate_count += 1
		customtkinter.CTkLabel(coordinate_frame, text=f"{coordinate_count}: X={vals[0]}, Y={vals[1]}, Z={vals[2]}").pack(anchor="w",pady=2)


#----------------------------
#	final UI commands

button_add = customtkinter.CTkButton(addframe, text = "Add point", command = add_values)
button_add.pack(pady = 10)

def run():
	selected = mode.get()



	


button_run = customtkinter.CTkButton(master=frame_1 ,text = "Run", command= run)
button_run.pack(pady=10)



button_exit = customtkinter.CTkButton(master=frame_1 ,text = "Exit", command=app.destroy)
button_exit.pack(pady=10)


app.mainloop()



