# Senior Captstone

## Responsibilites and Overview of Contribution

* Team Lead: Organized team into an agile environment using the project management software Click-Up.

* Software and Controls Lead: Took existing software in ROS and rewrote libraries to better leverage third-party packages and streamline communications.

* Co-created Robotic Arm: assisted in design, analysis, manufacturing and wiring. Designed control code on Arduinos and in ROS to move arm with game-pad leveraging DC and stepper motor controllers.

![code diagram](figures/Blank_diagram.png)

### Figure 1: Code Diagram of Electronics and Data Flow

## Control System Overviews

### Communication and Controls

The communication scheme and connections for the Adventure rover are presented in Figure 1, which illustrates the data flow for the entire rover. As shown in the figure, the base station is connected to the rover's computer through a wireless communication system that utilizes the 900 MHz and 2.1 GHz bands.

### Drive Controls

The previous year's team used a HERO Development board, a CAN bus-enabled microcontroller, to operate the Talon SRX speed controllers and control the speed of the motors. However, the current team reduced the number of wheels from six to four, which led to a polarity reversal of two wheels and a 33% increase in voltage to the motors, causing an increase in wheel speed. The team addressed this issue by reworking the communication and code that sends signals to the Talon SRX speed controllers. Additionally, the team reworked the Simple DirectMedia Layer (SDL), a C++ library, to read logitech controller inputs to regulate the motors. This allowed for static assigning of controllers to prevent conflicting signals and ensured that all controllers could be connected in any order.

### Arm Controls

Since the arm was replaced with an in-house built arm, the motors and electronics required a complete overhaul. The new arm employs worm gear joints connected to 24V DC motors. Initially, these motors contained encoders that allowed for closed loop feedback control to track the motor's position. However, the encoders prevented full rotation of the arm, making it unsuitable for the arm's application. Therefore, an open-loop control design was implemented. The team utilized the joy framework within the Robotic Operating System (ROS) to obtain input from a logitech controller. The team created a ROS node on the base station that interprets the controller input and sends information in the form of 2, 0, and -2 to each motor, indicating the polarity and power of the motor. This information is picked up by an Arduino connected to the rover's computer, which sends a full power signal to the motor on input and decays it over half a second unless another signal is received.
