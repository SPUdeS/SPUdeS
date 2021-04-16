# Module `SPUdeS` {#SPUdeS}
## Sub-modules

* [SPUdeS.Arduino](#SPUdeS.Arduino)
* [SPUdeS.Platform](#SPUdeS.Platform)
* [SPUdeS.PythonUI](#SPUdeS.PythonUI)

# Module `SPUdeS.Arduino` {#SPUdeS.Arduino}

## Sub-modules

* [SPUdeS.Arduino.ard_communication](#SPUdeS.Arduino.ard_communication)

# Module [`SPUdeS.Arduino.ard_communication`](#SPUdeS.Arduino.ard_communication)

## Classes

### Class `ard_communication` {#SPUdeS.Arduino.ard_communication.ard_communication}

>     class ard_communication


The ard_communication class handles all the communication protocol between the Arduino and the Python
functions using the library pyfirmata.

The __init__() function sets up the connection to the Arduino via USB, it initializes the Motor to Pin
connections and some basic set up angles (min, max, homing angles). This function also calls the
setUpMotors() function to initialize the motor connections with the Arduino. It finally calls the
gotoHomePosition() function which sends the homing angles command to the motors.








#### Methods



##### Method `goToHomePosition` {#SPUdeS.Arduino.ard_communication.ard_communication.goToHomePosition}




>     def goToHomePosition(
>         self
>     )


The goToHomePosition() function calls the setServoAngle() function with the arguments of the homing angles
initialized in the __init__().


##### Method `setServoAngle` {#SPUdeS.Arduino.ard_communication.ard_communication.setServoAngle}




>     def setServoAngle(
>         self,
>         angle,
>         isRad=1
>     )


The setServoAngle() function receives a set of arrays with each 6 elements. The sets of angles are for
sending small movements to the motors and the 6 elements in each array are the angles for each one of the
motors. There is also an option for sending the list of angle arrays in radian or not. By default,
we assume that the angles are in radians.


##### Method `setUpMotors` {#SPUdeS.Arduino.ard_communication.ard_communication.setUpMotors}




>     def setUpMotors(
>         self
>     )


The setUpMotors() function initializes the pin connections to the servo motors.




# Module `SPUdeS.Platform` {#SPUdeS.Platform}





## Sub-modules

* [SPUdeS.Platform.config](#SPUdeS.Platform.config)
* [SPUdeS.Platform.kinematicFunctions](#SPUdeS.Platform.kinematicFunctions)
* [SPUdeS.Platform.stewartClasses](#SPUdeS.Platform.stewartClasses)
* [SPUdeS.Platform.stewartPlatform](#SPUdeS.Platform.stewartPlatform)







# Module `SPUdeS.Platform.config` {#SPUdeS.Platform.config}

Configuration file to define the geometry of the platform.
This file contain the relevant information to perform the inverse kinematics computations.
If the platform geometry is changed, then this file should be modified and everything else
should stay the same in the stewartPlatform file.








# Module `SPUdeS.Platform.kinematicFunctions` {#SPUdeS.Platform.kinematicFunctions}

This module defines various kinematic functions used in the stewartClasses and the stewartPlatform.





## Functions



### Function `getAlpha` {#SPUdeS.Platform.kinematicFunctions.getAlpha}




>     def getAlpha(
>         effectiveLegLength,
>         beta,
>         base,
>         platform
>     )


Computes servo angle as a function of the platform orientation and position,
the leg's effective length and it's angle beta (angle between servo arm and base x-axis).


### Function `getAnglesFromRotationMatrix` {#SPUdeS.Platform.kinematicFunctions.getAnglesFromRotationMatrix}




>     def getAnglesFromRotationMatrix(
>         b_R_p
>     )


Computes Yaw, pitch, roll from rotation matrix b_R_p.
Convention: Euler ZYX (α,β,γ).


### Function `getNumberOfWaypoints` {#SPUdeS.Platform.kinematicFunctions.getNumberOfWaypoints}




>     def getNumberOfWaypoints(
>         initialOrigin,
>         initialVectorBase,
>         targetOrigin,
>         targetVectorBase
>     )


Returns the number of samples.
Distance to travel sampled at a rate defined by config variable pathSamplingPrecision.


### Function `getRotationMatrix` {#SPUdeS.Platform.kinematicFunctions.getRotationMatrix}




>     def getRotationMatrix(
>         vb1,
>         vb2
>     )


Computes the rotation matrix between two vector bases.
Convention: Euler ZYX (α,β,γ).


### Function `getRotationMatrixFromAngles` {#SPUdeS.Platform.kinematicFunctions.getRotationMatrixFromAngles}




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





# Module `SPUdeS.Platform.stewartClasses` {#SPUdeS.Platform.stewartClasses}

This file contains the building blocks of the stewartPlatform class.
Contains the frame, _piece, base and platform classes.






## Classes



### Class `base` {#SPUdeS.Platform.stewartClasses.base}




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







### Class `frame` {#SPUdeS.Platform.stewartClasses.frame}




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



##### Method `getOrigin` {#SPUdeS.Platform.stewartClasses.frame.getOrigin}




>     def getOrigin(
>         self
>     )





##### Method `getVectorBase` {#SPUdeS.Platform.stewartClasses.frame.getVectorBase}




>     def getVectorBase(
>         self
>     )





##### Method `setOrigin` {#SPUdeS.Platform.stewartClasses.frame.setOrigin}




>     def setOrigin(
>         self,
>         origin
>     )





##### Method `setVectorBase` {#SPUdeS.Platform.stewartClasses.frame.setVectorBase}




>     def setVectorBase(
>         self,
>         vectorBase
>     )





##### Method `updateFrame` {#SPUdeS.Platform.stewartClasses.frame.updateFrame}




>     def updateFrame(
>         self,
>         newFrame
>     )





### Class `platform` {#SPUdeS.Platform.stewartClasses.platform}




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



##### Method `getAngles` {#SPUdeS.Platform.stewartClasses.platform.getAngles}




>     def getAngles(
>         self
>     )


Returns the angles of the platform based on its rotation matrix / vector base.


##### Method `getHomeOrigin` {#SPUdeS.Platform.stewartClasses.platform.getHomeOrigin}




>     def getHomeOrigin(
>         self,
>         linkedBase
>     )


Returns home position of platform - computed from base/platform anchors #1 but any would do by symmetry.




# Module `SPUdeS.Platform.stewartPlatform` {#SPUdeS.Platform.stewartPlatform}

This file contains the stewartPlatform class. This defines the geometry of the stewart platform.






## Classes



### Class `stewartPlatform` {#SPUdeS.Platform.stewartPlatform.stewartPlatform}




>     class stewartPlatform


Contains inner classes base and platform. This defines the geometry of the stewart platform.







#### Static methods



##### `Method confirmRequestValidity` {#SPUdeS.Platform.stewartPlatform.stewartPlatform.confirmRequestValidity}




>     def confirmRequestValidity(
>         type_,
>         data_
>     )


Confirms whether or not the request is valid.
Returns 0 if type is target, 1 if type is sweep, and -1 if there is a problem with the request.


##### `Method drawLinesBetweenPoints` {#SPUdeS.Platform.stewartPlatform.stewartPlatform.drawLinesBetweenPoints}




>     def drawLinesBetweenPoints(
>         axis,
>         listOfPointsToJoin,
>         colour
>     )


This function takes a plot and a list of points to join as parameters.
The colour parameter specifies the colour of the line to draw.


##### `Method getTargetFrameFromPoint` {#SPUdeS.Platform.stewartPlatform.stewartPlatform.getTargetFrameFromPoint}




>     def getTargetFrameFromPoint(
>         targetPoint
>     )


Returns a frame for a target point.



#### Methods



##### Method `UpdateArmPoints` {#SPUdeS.Platform.stewartPlatform.stewartPlatform.UpdateArmPoints}




>     def UpdateArmPoints(
>         self
>     )


Initiates useful variables to plot the stewart platform.


##### Method `display3D` {#SPUdeS.Platform.stewartPlatform.stewartPlatform.display3D}




>     def display3D(
>         self,
>         updateArmPoints=False
>     )


This function plots the main 3D plot of the stewartPlatform
Redefines the 3D png plot found in the PythonUI module.
Variable updateArmPoints defaults to false and specifies if the arms are to be recalculated.


##### Method `displayView` {#SPUdeS.Platform.stewartPlatform.stewartPlatform.displayView}




>     def displayView(
>         self,
>         updateArmPoints=False
>     )


This function plots three views of the current position of the stewart platform.
Redefines the three 2D png plots found in the PythonUI module.
Variable updateArmPoints defaults to false and specifies if the arms are to be recalculated.


##### Method `generateListOfTargets` {#SPUdeS.Platform.stewartPlatform.stewartPlatform.generateListOfTargets}




>     def generateListOfTargets(
>         self,
>         requestType,
>         data_
>     )


Redirects request depending on its type.


##### Method `getArmPoints` {#SPUdeS.Platform.stewartPlatform.stewartPlatform.getArmPoints}




>     def getArmPoints(
>         self
>     )





##### Method `getArmPointsToJoin` {#SPUdeS.Platform.stewartPlatform.stewartPlatform.getArmPointsToJoin}




>     def getArmPointsToJoin(
>         self
>     )


Helper function for the plot that defines the points that are to join for the servo arms.


##### Method `getBasePlatformRotationMatrix` {#SPUdeS.Platform.stewartPlatform.stewartPlatform.getBasePlatformRotationMatrix}




>     def getBasePlatformRotationMatrix(
>         self
>     )


Returns the rotation matrix between the base and the platform.


##### Method `getEffectiveLegLengths` {#SPUdeS.Platform.stewartPlatform.stewartPlatform.getEffectiveLegLengths}




>     def getEffectiveLegLengths(
>         self,
>         targetFrame
>     )


Compute effective leg lengths : Effective length = T + b_R_p * Pi - Bi.


##### Method `getLegPointsToJoin` {#SPUdeS.Platform.stewartPlatform.stewartPlatform.getLegPointsToJoin}




>     def getLegPointsToJoin(
>         self
>     )


Helper function for the plot that defines the points that are to join to the platform.


##### Method `getListServoAngles` {#SPUdeS.Platform.stewartPlatform.stewartPlatform.getListServoAngles}




>     def getListServoAngles(
>         self,
>         listOfTargets
>     )


Return a list of six-tuples servo angles to move through the trajectory.
Returns the last waypoint to later update the platform's origin.


##### Method `getRotationAnglesToFrame` {#SPUdeS.Platform.stewartPlatform.stewartPlatform.getRotationAnglesToFrame}




>     def getRotationAnglesToFrame(
>         self,
>         targetFrame
>     )


Returns the rotation angle between the platform and a target frame.


##### Method `getRotationMatrixToTarget` {#SPUdeS.Platform.stewartPlatform.stewartPlatform.getRotationMatrixToTarget}




>     def getRotationMatrixToTarget(
>         self,
>         targetVectorBase
>     )


Returns the rotation matrix between the base and a target frame.


##### Method `getServoAngles` {#SPUdeS.Platform.stewartPlatform.stewartPlatform.getServoAngles}




>     def getServoAngles(
>         self
>     )





##### Method `initHomeServoAngle` {#SPUdeS.Platform.stewartPlatform.stewartPlatform.initHomeServoAngle}




>     def initHomeServoAngle(
>         self
>     )


This function defines the servo six servo angles at the home position.


##### Method `inverseKinematics` {#SPUdeS.Platform.stewartPlatform.stewartPlatform.inverseKinematics}




>     def inverseKinematics(
>         self,
>         targetFrame
>     )


Takes in parameters the new target frame the user wants to go to and the actual
frame (given by the object platform).
Returns servo angles necessary to accomplish the movement.
target = [x, y, z, ψ, θ, φ ] (angles d'Euler)


##### Method `pathSampling` {#SPUdeS.Platform.stewartPlatform.stewartPlatform.pathSampling}




>     def pathSampling(
>         self,
>         targetFrame
>     )


Return a list of waypoints to get to the target.
Each waypoint is a new frame coinciding with the new platform location
Uses precision from config file


##### Method `requestFromFlask` {#SPUdeS.Platform.stewartPlatform.stewartPlatform.requestFromFlask}




>     def requestFromFlask(
>         self,
>         type_,
>         data_
>     )


This function handles an initialization, displacement or sweep request from flask.
Returns a list of servo angles.


##### Method `requestShowoffFromFlask` {#SPUdeS.Platform.stewartPlatform.stewartPlatform.requestShowoffFromFlask}




>     def requestShowoffFromFlask(
>         self
>     )


This function handles a showoff request from flask. Defines a complex trajectory.
Returns a list of servo angles.


##### Method `targetListForInitialize` {#SPUdeS.Platform.stewartPlatform.stewartPlatform.targetListForInitialize}




>     def targetListForInitialize(
>         self
>     )


Defines the list of targets in an initialization request.


##### Method `targetListForSweep` {#SPUdeS.Platform.stewartPlatform.stewartPlatform.targetListForSweep}




>     def targetListForSweep(
>         self,
>         DoF
>     )


Defines the list of targets in a sweep request.


##### Method `targetListForTarget` {#SPUdeS.Platform.stewartPlatform.stewartPlatform.targetListForTarget}




>     def targetListForTarget(
>         self,
>         displacements
>     )


Defines the list of targets in a simple displacement request.


##### Method `updateAllPlots` {#SPUdeS.Platform.stewartPlatform.stewartPlatform.updateAllPlots}




>     def updateAllPlots(
>         self,
>         updateArmPoints=True
>     )


This functions runs both plot generating function.
This has the advantage of only updating the arm points once.
Variable updateArmPoints defaults to true and specifies if the arms are to be recalculated.




# Module [`SPUdeS.PythonUI`](#SPUdeS.PythonUI)





## Sub-modules

* [SPUdeS.PythonUI.Server](#SPUdeS.PythonUI.Server)
* [SPUdeS.PythonUI.config](#SPUdeS.PythonUI.config)







# Module `SPUdeS.PythonUI.Server` {#SPUdeS.PythonUI.Server}

This Server.py file is a class called Server() that sets up the server and runs it. This is the link between the client side inputs and the cinematic
functions to send commands and new graphs.






## Classes



### Class `Server` {#SPUdeS.PythonUI.Server.Server}




>     class Server


Web server class.

The __init__ function instantiates a stewartPlatform(), an ard_communication() and
a Flask object. This function also sets up the home plot for the 3D graphs and sets up a camera. It then initializes
the routes for the Flask server and runs the server.







#### Static methods



##### `Method initPlot` {#SPUdeS.PythonUI.Server.Server.initPlot}




>     def initPlot()


initPlot() refactors the 3D plots created to give us a clean slate to work with. It re-initializes the plots
to the default ones. The plots are in SPUdeS/PythonUI/static/img.



#### Methods



##### Method `generate_frames` {#SPUdeS.PythonUI.Server.Server.generate_frames}




>     def generate_frames(
>         self
>     )


generate_frame() function generates the frame by frame of the camera to create the live feed on the web page.


##### Method `initiateFlaskApp` {#SPUdeS.PythonUI.Server.Server.initiateFlaskApp}




>     def initiateFlaskApp(
>         self
>     )


The initiateFlaskApp() function calls an instance of the Flask object.


##### Method `requestSP` {#SPUdeS.PythonUI.Server.Server.requestSP}




>     def requestSP(
>         self,
>         type_,
>         data
>     )


requestSP() is called through the /NewDisplacementRequest route. It uses the json payload sent from the JavaScript function to send a command in angles to the Arduino
and to update the 3D plots.


##### Method `requestShowoffSP` {#SPUdeS.PythonUI.Server.Server.requestShowoffSP}




>     def requestShowoffSP(
>         self
>     )


requestShowoffSP() is called through the /NewShowoffRequest route. It calls a function to send predetermined movement commands to the Arduino
and to update the 3D plots.


##### Method `run` {#SPUdeS.PythonUI.Server.Server.run}




>     def run(
>         self
>     )


The run() function runs an instance of the Flask server called app.


##### Method `updateCamera` {#SPUdeS.PythonUI.Server.Server.updateCamera}




>     def updateCamera(
>         self,
>         cameraNumber=0
>     )


updateCamera() has by default value 0 for the number of the camera to use on the server. If a camera change is requested cameraNumber will
change to the number requested. cv2 is the imported library for OpenCV which allows us to have a live feed.




# Module `SPUdeS.PythonUI.config` {#SPUdeS.PythonUI.config}

Configuration File for the PythonUI folder. This file shares the path of PythonUI to be able to store the images
created in the kinematic functions






-----
Generated by *pdoc* 0.9.2 (<https://pdoc3.github.io>).

PDF-ready markdown written to standard output.
                              ^^^^^^^^^^^^^^^
Convert this file to PDF using e.g. Pandoc:

    pandoc --metadata=title:"MyProject Documentation"               \
           --from=markdown+abbreviations+tex_math_single_backslash  \
           --pdf-engine=xelatex --variable=mainfont:"DejaVu Sans"   \
           --toc --toc-depth=4 --output=pdf.pdf  pdf.md

or using Python-Markdown and Chrome/Chromium/WkHtmlToPDF:

    markdown_py --extension=meta         \
                --extension=abbr         \
                --extension=attr_list    \
                --extension=def_list     \
                --extension=fenced_code  \
                --extension=footnotes    \
                --extension=tables       \
                --extension=admonition   \
                --extension=smarty       \
                --extension=toc          \
                pdf.md > pdf.html

    chromium --headless --disable-gpu --print-to-pdf=pdf.pdf pdf.html

    wkhtmltopdf --encoding utf8 -s A4 --print-media-type pdf.html pdf.pdf

or similar, at your own discretion.

