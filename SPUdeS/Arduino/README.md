# <a id="SPUdeS.Arduino"></a> Module `SPUdeS.Arduino`

## Sub-modules

* [SPUdeS.Arduino.ard_communication](#SPUdeS.Arduino.ard_communication)


# <a id="SPUdeS.Arduino.ard_communication"></a> Module `SPUdeS.Arduino.ard_communication`

## Classes

### <a id="SPUdeS.Arduino.ard_communication.ard_communication"></a> Class `ard_communication`

>     class ard_communication


The ard_communication class handles all the communication protocol between the Arduino and the Python
functions using the library pyfirmata.

The __init__() function sets up the connection to the Arduino via USB, it initializes the Motor to Pin
connections and some basic set up angles (min, max, homing angles). This function also calls the
setUpMotors() function to initialize the motor connections with the Arduino. It finally calls the
gotoHomePosition() function which sends the homing angles command to the motors.



#### Methods



##### <a id="SPUdeS.Arduino.ard_communication.ard_communication.goToHomePosition"></a> Method `goToHomePosition`




>     def goToHomePosition(
>         self
>     )


The goToHomePosition() function calls the setServoAngle() function with the arguments of the homing angles
initialized in the __init__().


##### <a id="SPUdeS.Arduino.ard_communication.ard_communication.setServoAngle"></a> Method `setServoAngle`




>     def setServoAngle(
>         self,
>         angle,
>         isRad=1
>     )


The setServoAngle() function receives a set of arrays with each 6 elements. The sets of angles are for
sending small movements to the motors and the 6 elements in each array are the angles for each one of the
motors. There is also an option for sending the list of angle arrays in radian or not. By default,
we assume that the angles are in radians.


##### <a id="SPUdeS.Arduino.ard_communication.ard_communication.setUpMotors"></a> Method `setUpMotors`




>     def setUpMotors(
>         self
>     )


The setUpMotors() function initializes the pin connections to the servo motors.

