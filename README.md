# indoor-localization
This project provides modules for locating / navigating devices by
means of inertial and magnetic sensor fusion.   

Current progress: a camera based indoor localization was provided by Yu Hu: 
https://github.com/yh570/indoor_localization_with_CV

Two options for future implenmentation: when camera can't capture the object or QR code, using sensors instead; Kalman filter evaluate from camera based and sensor based

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

Collected Data: https://drive.google.com/drive/u/0/folders/0B6WOejI_hzK7aGl2U0RGYmFwLTA
