motiveAPI folder contains c++ application for reading motive stream and outputting to local twisted server
	-make sure motive usb is plugged in and motive application is actually streaming data otherwise this streaming will not work

Order to properly start streaming to raspberry pi: run c++ application (after motive is streaming) and then start twisted server

script folder contains python stuff for the twisted server
	-twistedServer.py is the twisted server
	-sampleguiclient_twisted.py is an example of using the qt4 gui library within twisted
	-other files are miscellaneous testing stuff

raspberry pi folder contains files from raspberry pi