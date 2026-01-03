Okay so this is a basic read me for internal purposes only

Call app.py to execute the program

The GCS has 2 major parts:
1. UI (Front End)
2. Core (Back End)

1. UI:
a. main_ui.py: Main (self-explanatory)
b. ui/dashboard.py: CTk Frame that contains all the other panels/frames
c. ui/(everything else): Self-explanatory 
d. ui/telemtry_view.py: Logs frames (No it is not for telemetry but changing the name will break everything)

2. Core:
a. core/gcs_controller.py: The main controller - interface between radio and ui
b. core/drone_state.py: Contains data class and enum for drone states
c. core/message_parser.py: JSON <-> Dict converter
d. core/radio_comm.py: Self-explanatory
e. core/telemetry_parser.py: Converts telemetry dictionary to formatted readable string for the UI

3. utils:
    a. helpers: Has helpers
    b. logger: Does logging

To change the radio serial port, update it in core/gcs_controller.py - the port in radio_comm.py is a default initial value
