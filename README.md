**Autonomous Sumo Robot | C++, Arduino, PlatformIO**

* **Objective:** Design and implementation of the software for a mini-sumo robot capable of detecting opponents and navigating autonomously within a competition ring.
* **Software Architecture:** Developed control logic based on a state machine (Search and Combat) to manage the robot's behavior depending on the presence of an opponent.
* **Hardware Integration:** Implemented real-time data processing routines for 5 infrared (IR) sensors for 360° opponent detection and 2 analog line sensors to prevent the robot from exiting the ring.
* **Abstraction and Modularity:** Created a custom library (`XMotionClass`) to abstract hardware components, handling motor control via PWM signals, sensor monitoring, and status signaling via LEDs.
* **Technologies used:** C++ programming on the Atmel AVR platform (Arduino Leonardo), using the PlatformIO environment for dependency management and the build process.
