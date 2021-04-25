# PiCNiC

PiCNiC is an open source program which allows you to turn your raspberry pi into a 3D CNC controller.

## General information
Your raspberry pi is not able to directly drive stepper motors, it can only be used to control the motor drivers. Per default, PiCNiC is developed to drive a 3-axis CNC machine, with one motor for x and z direction each and two motors in y-direction using DRV8825 stepper motor drivers. Furthermore, a MCP23017-E/SP port expander is used (using the I2C protocol) since the raspberry pi does not provide sufficient GPIO ports for endswitches, relays, microstepping controls etc.

## Features:
    -control of 4 motors (two motors for y-axis)
    -endswitches
    -automated start/stop of millig motor
    -fan control
    -parsing of svg and stl files
    -multiple milling strategies (e.g. contour, constant z, parallel)
    -manual controls of all motors
 
 ## Requirements and dependencies
    The code was tested on raspberry pi 3 and 4
    PiCNiC requires the following modules to be installed on your raspberry pi:
        - pigpio
        - PyQt5
        - matplotlib
        - scipy
        - trimesh
        - numpy-stl
        - shapely
        - pyyaml
 ## Usage
 ### SVG import
 #### Contour
 <img width="1028" alt="grafik" src="https://user-images.githubusercontent.com/19652477/115993417-f65df500-a5d2-11eb-871e-8e5a37e02bc1.png">
 
 #### Constant Z
 
 ### STL import
 
 #### Scan
 #### Contour
 
 ## Credits
  The icons included in this repository designed by freepik from Flaticon
  PiCNiC uses third party libraries for the communication with the MCP23017 provided by 
  https://bitbucket.org/dewoodruff/mcp23017-python-3-library-with-interrupts/src/master/
  
## License
  MIT
