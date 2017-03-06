Raspberry Pi Python Image Processor

These files make up the vision processing portion of the robot.  We'll need to
modify the /etc/rc.d/rc.local file on the raspberry pi in order to automatically
run piVisionMain on startup.

The piVisionMain file calls the vision processing methods and the network
communication methods - but these methods will live in different files.

For testing, change the roboRioIP variable in piVisionMain to point to 127.0.0.1
and run both piVisionMain and roboRioTest.  The latter will just print the
values received over the network, so you can verify that the data is being sent
properly.
