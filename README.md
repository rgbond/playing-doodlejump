# playing-doodlejump
A ROS workspace used to manage a robot that plays DoodleJump on an iPhone.
The setup includes an IPhone, an Ximea camera focused on the iPhone screen,
a stepper motor used to rotate the iPhone, a servo and a solenoid used to
press buttons on the screen, and an Nvidia Jetson running L4T, ROS and Tensorflow.

Major ROS components:
    * doobot
    * game
    * implay
    * score
    * xiq
    * viz

Doobot is a python script that controls the motors and servos on the robot. All
it does is decode mesaages from the game compoent and send the EIBot commands.
The scripts looks for an EIBot board on /dev/TTY/AM*.  The port seems to change
from release to release depending on the USB serial packaging so you may have
to tweak this code a bit.

Game is the heart of the setup. It includes two major pieces: archive.py
and db.py. Archive.py runs the game. It archives camera images, and commands
from the user and the inference engines. It controls the actions of the Tensorflow
components in implay. All achived data is sent to an sqlite database. The
file db.py includes a set of simple routines used to manage the database. 
The database and the archived images are normally stored at the top
level of the ROS tree in the db directory.

Implay is the component runnning the Tensorflow AI engine. It watches
for messages coming from the camera and hands them to the inference
engine to decide which actions to take. The actions are fed back to 
the game component. There are two versions here. implay/src has an old
Caffe/Tensorrt setup. It was replaced by the implay/scripts Tensorflow
verison. I left the Caffe setup in place as a reference. On command
from archive.py, images are pulled out of the database, rewards
are calculaged, and a training run is preformed. See scripts/sts.py
for the details.

Score is a special purpose Caffe/Tensorrt neural netowrk used to 
pull the DoodleJump score out of the images from the camera. It recognizes
the digits in the top left part of the screen and passes them along with
the frame number back to the game componet to be archived in the database.
The neural net was trained in Caffe. It is pretty good, but gets things
wrong now and then, mostly beause the screen is constantly changing. So
the scores have to be filtered before they can be converted to a reward.

Xiq is my renamed version of the ROS ximea_camera sensors module from the
University of Waterloo. Just install it here and it should work fine with
a couple of tweaks to the launch scripts. I can help or send my hacks if
you run into problems.

Viz is a simple module used to display images from the camera in a bounding
box used to manually align, focus, zoom and set the aperture on the camera. 

Archiving a full ROS workspace is not ideal. It may make sense to break
out each of the major subcomponents into their own tree at some point
in the future as the project matures.
