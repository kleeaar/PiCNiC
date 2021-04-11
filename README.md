#PiCNiC

PiCNiC is a open source program which allows you to turn your raspberry pi into a 3D CNC controller.

##General information
Your raspberry pi is not able to directly drive stepper motors, it can only be used to control the motor drivers. Per default, PiCNiC is developed to drive a 3-axis CNC machine, with each one x/z motors and two motors in y direction using DRV8825 stepper motor drivers. Furthermore, a MCP23017-E/SP port expander is used (using the I2C protocol) since the raspberry pi does not provide sufficient ports for endswitches, relays, microstepping controls etc.

##Features:
    -control of 4 motors (two motors for y-axis)
    -endswitches
    -automated start/stop of millig motor
    -fan control
    -parsing of svg and stl files
    -multiple milling strategies (e.g. contour, constant z, parallel)
