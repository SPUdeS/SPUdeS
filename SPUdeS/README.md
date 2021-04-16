# <a id="SPUdeS"></a> Module `SPUdeS`
## Sub-modules

* [SPUdeS.Arduino](#SPUdeS.Arduino)
* [SPUdeS.Platform](#SPUdeS.Platform)
* [SPUdeS.PythonUI](#SPUdeS.PythonUI)

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




# <a id="SPUdeS.Platform"></a> Module `SPUdeS.Platform`





## Sub-modules

* [SPUdeS.Platform.config](#SPUdeS.Platform.config)
* [SPUdeS.Platform.kinematicFunctions](#SPUdeS.Platform.kinematicFunctions)
* [SPUdeS.Platform.stewartClasses](#SPUdeS.Platform.stewartClasses)
* [SPUdeS.Platform.stewartPlatform](#SPUdeS.Platform.stewartPlatform)







# <a id="SPUdeS.Platform.config"></a> Module `SPUdeS.Platform.config`

Configuration file to define the geometry of the platform.
This file contain the relevant information to perform the inverse kinematics computations.
If the platform geometry is changed, then this file should be modified and everything else
should stay the same in the stewartPlatform file.








# <a id="SPUdeS.Platform.kinematicFunctions"></a> Module `SPUdeS.Platform.kinematicFunctions`

This module defines various kinematic functions used in the stewartClasses and the stewartPlatform.





## Functions



### <a id="SPUdeS.Platform.kinematicFunctions.getAlpha"></a> Function `getAlpha`




>     def getAlpha(
>         effectiveLegLength,
>         beta,
>         base,
>         platform
>     )


Computes servo angle as a function of the platform orientation and position,
the leg's effective length and it's angle beta (angle between servo arm and base x-axis).


### <a id="SPUdeS.Platform.kinematicFunctions.getAnglesFromRotationMatrix"></a> Function `getAnglesFromRotationMatrix`




>     def getAnglesFromRotationMatrix(
>         b_R_p
>     )


Computes Yaw, pitch, roll from rotation matrix b_R_p.
Convention: Euler ZYX (α,β,γ).


### <a id="SPUdeS.Platform.kinematicFunctions.getNumberOfWaypoints"></a> Function `getNumberOfWaypoints`




>     def getNumberOfWaypoints(
>         initialOrigin,
>         initialVectorBase,
>         targetOrigin,
>         targetVectorBase
>     )


Returns the number of samples.
Distance to travel sampled at a rate defined by config variable pathSamplingPrecision.


### <a id="SPUdeS.Platform.kinematicFunctions.getRotationMatrix"></a> Function `getRotationMatrix`




>     def getRotationMatrix(
>         vb1,
>         vb2
>     )


Computes the rotation matrix between two vector bases.
Convention: Euler ZYX (α,β,γ).


### <a id="SPUdeS.Platform.kinematicFunctions.getRotationMatrixFromAngles"></a> Function `getRotationMatrixFromAngles`




>     def getRotationMatrixFromAngles(
>         alpha,
>         beta,
>         gamma
>     )


Sets the rotation matrix needed given yaw, pitch and roll.
Based on Yaw, pitch, roll - Euler ZYX(alpha, beta, gamma) convention.
platform unitvectors are given as: [[Px],[Py],[Pz]] in reference frame Base
alpha = angle around z
beta = angle around y
gamma = angle around x





# <a id="SPUdeS.Platform.stewartClasses"></a> Module `SPUdeS.Platform.stewartClasses`

This file contains the building blocks of the stewartPlatform class.
Contains the frame, _piece, base and platform classes.






## Classes



### <a id="SPUdeS.Platform.stewartClasses.base"></a> Class `base`




>     class base(
>         origin=[0, 0, 0],
>         vectorBase=array([[1, 0, 0],
       [0, 1, 0],
       [0, 0, 1]])
>     )


This class defines the properties of a base piece. Extends _piece class.
This class is instantiated in the stewartPlatform class.



#### Ancestors (in MRO)

* [SPUdeS.Platform.stewartClasses._piece](#SPUdeS.Platform.stewartClasses._piece)
* [SPUdeS.Platform.stewartClasses.frame](#SPUdeS.Platform.stewartClasses.frame)







### <a id="SPUdeS.Platform.stewartClasses.frame"></a> Class `frame`




>     class frame(
>         origin,
>         vectorBase
>     )


Position : column vector describing position in Base Frame
Unit vectors [[Nx],[Ny],[Nz]] where Nx,y,z are column vectors
representing unitvectors of the frame in the Base reference frame




#### Descendants

* [SPUdeS.Platform.stewartClasses._piece](#SPUdeS.Platform.stewartClasses._piece)






#### Methods



##### <a id="SPUdeS.Platform.stewartClasses.frame.getOrigin"></a> Method `getOrigin`




>     def getOrigin(
>         self
>     )





##### <a id="SPUdeS.Platform.stewartClasses.frame.getVectorBase"></a> Method `getVectorBase`




>     def getVectorBase(
>         self
>     )





##### <a id="SPUdeS.Platform.stewartClasses.frame.setOrigin"></a> Method `setOrigin`




>     def setOrigin(
>         self,
>         origin
>     )





##### <a id="SPUdeS.Platform.stewartClasses.frame.setVectorBase"></a> Method `setVectorBase`




>     def setVectorBase(
>         self,
>         vectorBase
>     )





##### <a id="SPUdeS.Platform.stewartClasses.frame.updateFrame"></a> Method `updateFrame`




>     def updateFrame(
>         self,
>         newFrame
>     )





### <a id="SPUdeS.Platform.stewartClasses.platform"></a> Class `platform`




>     class platform(
>         linkedBase,
>         origin=[0, 0, 0],
>         vectorBase=array([[1, 0, 0],
       [0, 1, 0],
       [0, 0, 1]])
>     )


This class defines the properties of a platform piece. Extends _piece class.
This class is instantiated in the stewartPlatform class.



#### Ancestors (in MRO)

* [SPUdeS.Platform.stewartClasses._piece](#SPUdeS.Platform.stewartClasses._piece)
* [SPUdeS.Platform.stewartClasses.frame](#SPUdeS.Platform.stewartClasses.frame)







#### Methods



##### <a id="SPUdeS.Platform.stewartClasses.platform.getAngles"></a> Method `getAngles`




>     def getAngles(
>         self
>     )


Returns the angles of the platform based on its rotation matrix / vector base.


##### <a id="SPUdeS.Platform.stewartClasses.platform.getHomeOrigin"></a> Method `getHomeOrigin`




>     def getHomeOrigin(
>         self,
>         linkedBase
>     )


Returns home position of platform - computed from base/platform anchors #1 but any would do by symmetry.




# <a id="SPUdeS.Platform.stewartPlatform"></a> Module `SPUdeS.Platform.stewartPlatform`

This file contains the stewartPlatform class. This defines the geometry of the stewart platform.






## Classes



### <a id="SPUdeS.Platform.stewartPlatform.stewartPlatform"></a> Class `stewartPlatform`




>     class stewartPlatform


Contains inner classes base and platform. This defines the geometry of the stewart platform.







#### Static methods



##### <a id="SPUdeS.Platform.stewartPlatform.stewartPlatform.confirmRequestValidity"></a> Method `confirmRequestValidity`




>     def confirmRequestValidity(
>         type_,
>         data_
>     )


Confirms whether or not the request is valid.
Returns 0 if type is target, 1 if type is sweep, and -1 if there is a problem with the request.


##### <a id="SPUdeS.Platform.stewartPlatform.stewartPlatform.drawLinesBetweenPoints"></a> Method `drawLinesBetweenPoints`




>     def drawLinesBetweenPoints(
>         axis,
>         listOfPointsToJoin,
>         colour
>     )


This function takes a plot and a list of points to join as parameters.
The colour parameter specifies the colour of the line to draw.


##### <a id="SPUdeS.Platform.stewartPlatform.stewartPlatform.getTargetFrameFromPoint"></a> Method `getTargetFrameFromPoint`




>     def getTargetFrameFromPoint(
>         targetPoint
>     )


Returns a frame for a target point.



#### Methods



##### <a id="SPUdeS.Platform.stewartPlatform.stewartPlatform.UpdateArmPoints"></a> Method `UpdateArmPoints`




>     def UpdateArmPoints(
>         self
>     )


Initiates useful variables to plot the stewart platform.


##### <a id="SPUdeS.Platform.stewartPlatform.stewartPlatform.display3D"></a> Method `display3D`




>     def display3D(
>         self,
>         updateArmPoints=False
>     )


This function plots the main 3D plot of the stewartPlatform
Redefines the 3D png plot found in the PythonUI module.
Variable updateArmPoints defaults to false and specifies if the arms are to be recalculated.


##### <a id="SPUdeS.Platform.stewartPlatform.stewartPlatform.displayView"></a> Method `displayView`




>     def displayView(
>         self,
>         updateArmPoints=False
>     )


This function plots three views of the current position of the stewart platform.
Redefines the three 2D png plots found in the PythonUI module.
Variable updateArmPoints defaults to false and specifies if the arms are to be recalculated.


##### <a id="SPUdeS.Platform.stewartPlatform.stewartPlatform.generateListOfTargets"></a> Method `generateListOfTargets`




>     def generateListOfTargets(
>         self,
>         requestType,
>         data_
>     )


Redirects request depending on its type.


##### <a id="SPUdeS.Platform.stewartPlatform.stewartPlatform.getArmPoints"></a> Method `getArmPoints`




>     def getArmPoints(
>         self
>     )





##### <a id="SPUdeS.Platform.stewartPlatform.stewartPlatform.getArmPointsToJoin"></a> Method `getArmPointsToJoin`




>     def getArmPointsToJoin(
>         self
>     )


Helper function for the plot that defines the points that are to join for the servo arms.


##### <a id="SPUdeS.Platform.stewartPlatform.stewartPlatform.getBasePlatformRotationMatrix"></a> Method `getBasePlatformRotationMatrix`




>     def getBasePlatformRotationMatrix(
>         self
>     )


Returns the rotation matrix between the base and the platform.


##### <a id="SPUdeS.Platform.stewartPlatform.stewartPlatform.getEffectiveLegLengths"></a> Method `getEffectiveLegLengths`




>     def getEffectiveLegLengths(
>         self,
>         targetFrame
>     )


Compute effective leg lengths : Effective length = T + b_R_p * Pi - Bi.


##### <a id="SPUdeS.Platform.stewartPlatform.stewartPlatform.getLegPointsToJoin"></a> Method `getLegPointsToJoin`




>     def getLegPointsToJoin(
>         self
>     )


Helper function for the plot that defines the points that are to join to the platform.


##### <a id="SPUdeS.Platform.stewartPlatform.stewartPlatform.getListServoAngles"></a> Method `getListServoAngles`




>     def getListServoAngles(
>         self,
>         listOfTargets
>     )


Return a list of six-tuples servo angles to move through the trajectory.
Returns the last waypoint to later update the platform's origin.


##### <a id="SPUdeS.Platform.stewartPlatform.stewartPlatform.getRotationAnglesToFrame"></a> Method `getRotationAnglesToFrame`




>     def getRotationAnglesToFrame(
>         self,
>         targetFrame
>     )


Returns the rotation angle between the platform and a target frame.


##### <a id="SPUdeS.Platform.stewartPlatform.stewartPlatform.getRotationMatrixToTarget"></a> Method `getRotationMatrixToTarget`




>     def getRotationMatrixToTarget(
>         self,
>         targetVectorBase
>     )


Returns the rotation matrix between the base and a target frame.


##### <a id="SPUdeS.Platform.stewartPlatform.stewartPlatform.getServoAngles"></a> Method `getServoAngles`




>     def getServoAngles(
>         self
>     )





##### <a id="SPUdeS.Platform.stewartPlatform.stewartPlatform.initHomeServoAngle"></a> Method `initHomeServoAngle`




>     def initHomeServoAngle(
>         self
>     )


This function defines the servo six servo angles at the home position.


##### <a id="SPUdeS.Platform.stewartPlatform.stewartPlatform.inverseKinematics"></a> Method `inverseKinematics`




>     def inverseKinematics(
>         self,
>         targetFrame
>     )


Takes in parameters the new target frame the user wants to go to and the actual
frame (given by the object platform).
Returns servo angles necessary to accomplish the movement.
target = [x, y, z, ψ, θ, φ ] (angles d'Euler)


##### <a id="SPUdeS.Platform.stewartPlatform.stewartPlatform.pathSampling"></a> Method `pathSampling`




>     def pathSampling(
>         self,
>         targetFrame
>     )


Return a list of waypoints to get to the target.
Each waypoint is a new frame coinciding with the new platform location
Uses precision from config file


##### <a id="SPUdeS.Platform.stewartPlatform.stewartPlatform.requestFromFlask"></a> Method `requestFromFlask`




>     def requestFromFlask(
>         self,
>         type_,
>         data_
>     )


This function handles an initialization, displacement or sweep request from flask.
Returns a list of servo angles.


##### <a id="SPUdeS.Platform.stewartPlatform.stewartPlatform.requestShowoffFromFlask"></a> Method `requestShowoffFromFlask`




>     def requestShowoffFromFlask(
>         self
>     )


This function handles a showoff request from flask. Defines a complex trajectory.
Returns a list of servo angles.


##### <a id="SPUdeS.Platform.stewartPlatform.stewartPlatform.targetListForInitialize"></a> Method `targetListForInitialize`




>     def targetListForInitialize(
>         self
>     )


Defines the list of targets in an initialization request.


##### <a id="SPUdeS.Platform.stewartPlatform.stewartPlatform.targetListForSweep"></a> Method `targetListForSweep`




>     def targetListForSweep(
>         self,
>         DoF
>     )


Defines the list of targets in a sweep request.


##### <a id="SPUdeS.Platform.stewartPlatform.stewartPlatform.targetListForTarget"></a> Method `targetListForTarget`




>     def targetListForTarget(
>         self,
>         displacements
>     )


Defines the list of targets in a simple displacement request.


##### <a id="SPUdeS.Platform.stewartPlatform.stewartPlatform.updateAllPlots"></a> Method `updateAllPlots`




>     def updateAllPlots(
>         self,
>         updateArmPoints=True
>     )


This functions runs both plot generating function.
This has the advantage of only updating the arm points once.
Variable updateArmPoints defaults to true and specifies if the arms are to be recalculated.

# <a id="SPUdeS.PythonUI"></a> Module `SPUdeS.PythonUI`

## Sub-modules

* [SPUdeS.PythonUI.Server](#SPUdeS.PythonUI.Server)
* [SPUdeS.PythonUI.config](#SPUdeS.PythonUI.config)

# <a id="SPUdeS.PythonUI.Server"></a> Module `SPUdeS.PythonUI.Server`

This Server.py file is a class called Server() that sets up the server and runs it. This is the link between the client side inputs and the cinematic
functions to send commands and new graphs.






## Classes



### <a id="SPUdeS.PythonUI.Server.Server"></a> Class `Server`




>     class Server


Web server class.

The __init__ function instantiates a stewartPlatform(), an ard_communication() and
a Flask object. This function also sets up the home plot for the 3D graphs and sets up a camera. It then initializes
the routes for the Flask server and runs the server.







#### Static methods



##### <a id="SPUdeS.PythonUI.Server.Server.initPlot"></a> Method `initPlot`




>     def initPlot()


initPlot() refactors the 3D plots created to give us a clean slate to work with. It re-initializes the plots
to the default ones. The plots are in SPUdeS/PythonUI/static/img.



#### Methods



##### <a id="SPUdeS.PythonUI.Server.Server.generate_frames"></a> Method `generate_frames`




>     def generate_frames(
>         self
>     )


generate_frame() function generates the frame by frame of the camera to create the live feed on the web page.


##### <a id="SPUdeS.PythonUI.Server.Server.initiateFlaskApp"></a> Method `initiateFlaskApp`




>     def initiateFlaskApp(
>         self
>     )


The initiateFlaskApp() function calls an instance of the Flask object.


##### <a id="SPUdeS.PythonUI.Server.Server.requestSP"></a> Method `requestSP`




>     def requestSP(
>         self,
>         type_,
>         data
>     )


requestSP() is called through the /NewDisplacementRequest route. It uses the json payload sent from the JavaScript function to send a command in angles to the Arduino
and to update the 3D plots.


##### <a id="SPUdeS.PythonUI.Server.Server.requestShowoffSP"></a> Method `requestShowoffSP`




>     def requestShowoffSP(
>         self
>     )


requestShowoffSP() is called through the /NewShowoffRequest route. It calls a function to send predetermined movement commands to the Arduino
and to update the 3D plots.


##### <a id="SPUdeS.PythonUI.Server.Server.run"></a> Method `run`




>     def run(
>         self
>     )


The run() function runs an instance of the Flask server called app.


##### <a id="SPUdeS.PythonUI.Server.Server.updateCamera"></a> Method `updateCamera`




>     def updateCamera(
>         self,
>         cameraNumber=0
>     )


updateCamera() has by default value 0 for the number of the camera to use on the server. If a camera change is requested cameraNumber will
change to the number requested. cv2 is the imported library for OpenCV which allows us to have a live feed.




# <a id="SPUdeS.PythonUI.config"></a> Module `SPUdeS.PythonUI.config`

Configuration File for the PythonUI folder. This file shares the path of PythonUI to be able to store the images
created in the kinematic functions
