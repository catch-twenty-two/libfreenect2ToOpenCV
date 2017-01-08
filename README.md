# libfreenect2ToOpenCV

This project allows a Microsoft Xbox Kinect2 to stream content to the OpenCV libraries using libfreenect2 in Ubuntu Linux.  I created it after I couldn't find a ready made way to interface the 2 libraries together.  It requires OpenCV along with libfreenect2 to be installed and their corresponsing libraries and include folders to be availible in the system path.

The example contains a Canny filter, but can be replaced with anything else.  Calls are availlible to grab all output types of the Xbox Kinect2 including IR and Depth Images.

Created and compiled in Eclipse Mars CDT environment on Ubuntu 14.04 LTS.

Uses parts from this open CV forum answer:

http://answers.opencv.org/question/76468/opencvkinect-onekinect-for-windows-v2linuxlibfreenect2/
