# Small-Scale-Autonomous-Watering-System
This project is on a very small scale, arduino autonomous watering control system that uses a resistive moisture sensor and water flow sensor as feedback gains.

The system is made up of an Arduino which will be the controller of the system, water flow sensor as a feedback gain, dc-motor along with the MOSFET as the plant, and a moisture sensor as a closed loop feedback gain. The system works by measuring the moisture level of the soil and comparing that to a pre-set value. If the soil is considered dry the dc-motor, which is controlled by a IRFZ44N MOSFET, will pump water into the soil and using PID control methods that keeps the soil at a certain moisture level. 
