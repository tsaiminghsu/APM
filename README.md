# ArduCopter(3.2.1 or 3.3.3)
## Update and upgrade all existing packages
### $sudo apt-get update
### $sudo apt-get upgrade
### Note: The $ symbol is not typed by the user. This just denotes a new line in the terminal.
## Install OpenCV libraries and Packages
### $sudo apt-get install opencv-dev python-opencv
## Install package that allows for the use of Webcams
### $sudo apt-get install guvcview
### Note: This step is not necessary if using a Raspberry Pi Camera
### Install all packages for Mavlink
### $sudo apt-get install screen python-wxgtk2.8 python-matplotlib
### python-pip python-numpy
## Enable the use of the serial ports on the Raspberry Pi for communication with the Pixhawk
### $sudo nano /etc/inittab
### Note: This is not necessary if connected between devices via USB
### Navigate to the very last line of the text and add a # at the beginning of the line, this
comments out the last line which should look like this
### #T0:23:respawn:/sbin/getty -L ttyAMA0 115200 vt100
### Note: In order to type #, you cannot use SHIFT-3 but instead must use the \ key
### Press CTRL-X and then Y in order to sace the change made
## Install all the required libraries for Mavlink
### $sudo apt-get install screen python-wxgtk2.8 python-matplotlib
### python-opencv python-pip python-numpy python-dev libxml2-dev
### libxslt-dev
### $sudo pip install pymavlink
### $sudo pip install mavproxy
## Disable the OS control of the serial port
### $sudo raspi-config
### Navigate to Advanced Options and hit enter
### Navigate to A8 Serial and hit enter
### Navigate to Disable and hit enter
## Install Dronekit API
### $sudo pip install dronekit droneapi
## Reboot the Raspberry Pi to finalize the installation of all packages and libraries
### $sudo reboot
### When the Raspberry Pi fully reboots, everything is ready to run. Now the code can be tested by
### opening the File Manager and navigating to the /home/pi file location (or in which ever location the code
### was saved). Right click on the QuadrotorUAVCode.py and select Open With: Python 2 IDLE. Note that
the code will not run and OpenCV will not be recognized if Python 3 IDLE is used. Once Python 2 IDLE
and Python Shell are open, click Run in the IDLE.
