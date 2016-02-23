# indoor-localization
This project provides modules for locating / navigating devices by
means of inertial and magnetic sensor fusion.   

A step detector / counter is provided in pedometer.r2py, while
heading and device orientation can be derived from the 
OrientationFilter class.

Inside the IndoorNavigator class, a user's steps are combined
with their heading to yield their location as they walk around indoors.

Below is an example:

getsensor = dy_import_module("getsensor.r2py")
dy_import_module_symbols("indoor_navigator.r2py")

getsensor.start_sensing(1, 1)
nav = IndoorNavigator()

while True:
  # update the navigator
  nav.navigate(getsensors())
  # log the new position
  log(nav.get_coordinates(), '\n'))
