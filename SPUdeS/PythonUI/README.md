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
