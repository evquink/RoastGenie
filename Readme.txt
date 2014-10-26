RoastGenie

This project documents enhancements to the control system in the GeneCafe
Coffee roaster.  It adds PID control, batch label printing, data collection
and processing, and bean mass temperature measurement.

Folders in this repository

* Documentation
Various documentation files covering system design and operation of the
system.

* HeaterInterface
Eagle schematics and board files for hardware that controls the 120v heater.

* PlotGenieSW 
Files required to process the data files from the plot genie using a python
script and gnuplot.

* RoasterInterface
Eagle schematics and board files for hardware that interfaces the Diavolino to
the modified coffee roaster.

* RoastGeniePanel
2-D drawing file for the panel on the RoastGenie Controller.

* RoastGenieSW_Diavolino
Arduino files for the program that runs on the Diavolino for executing primary
roaster control.

* RoastGenieSW_ProTrinket
Arduino files for the program that runs on the ProTrinket for capturing bean
mass temperature.

* RoastGenieSW_RasPi
The Python script that does the data acquisition and file management for the
RoastGenie




