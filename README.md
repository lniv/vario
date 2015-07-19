# vario

A sailplane / soaring  variometer project.
Minimal use is as a standalone vario, using a pressure sensor (typically connected to a TE probe) to measure climb rate. The code running on a teensy 3.1  (using the teensyduino environment (but not the IDE (shudder)) drives an indicator gauge stepper and a speaker.
Optionally, a differential pressure sensor and a gps are connected to the uC, and information is sent to a flight computer.
Current protocol implemented is the open vario one, and the gps GPRMC sentences are just echoed.
Currently a 9dof imu is connected as well, but the data is not procssed. In the far future one could integrate barometric / imu / gps data to get a better state estimate.
 
