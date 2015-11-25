CS184 AS0

This is a simple OpenGL example program. See the instructional site for 
details on the assignment. This file outlines the method to compile this
program on various platforms.



Windows
-------
There is an included "CS184 Sp13 AS0.sln" file. Please open it with Visual
Studio 2012. Visual Studio 2012 can be obtained for free for students on
https://www.dreamspark.com. Select Visual Studio Professional 2012.

Once the .sln file is opened, you can press "F5" to build and run the program.



OS X
----
You will need to download XCode for gcc. You will not necessarily run XCode,
but installation of XCode will provide required packages for your computer.

XCode can be obtained on the App Store for free for OS X versions that have
the App Store available (Lion/10.7 and above)

If you are using an older version of OS X, you may need to find an appropriate
version of XCode to download. For example, students running Snow Leopard (10.6)
must register on Apple's developer network (a subscription is not required for
this file, but a registration is), and download the following file:
https://connect.apple.com/cgi-bin/WebObjects/MemberSite.woa/wa/getSoftware?bundleID=20792

If you are using Mountain Lion (10.8) or later, please download XQuartz/X11
following the instructions on the following webpage:
http://xquartz.macosforge.org/trac/wiki/Releases

Then, open a terminal and navigate to this directory. Run "make", then run
"./as0"



Linux
-----
You need to download freeglut and essential build programs. This varies from
distribution to distribution, but on distributions running "apt-get" such as
Ubuntu, this can be achieved by running:
sudo apt-get update; sudo apt-get install build-essential freeglut3-dev

Once the packages have been installed, open a terminal and navigate to this 
directory. Run "make", then run "./as0"
