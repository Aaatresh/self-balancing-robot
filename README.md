# Self Balancing Robot

<table>
  <tr>
    <td>  <img src="./images/side_view.jpg" width="300"/> </td>
    <td>  <img src="./images/front_view.jpg" width="300"/> </td>
  </tr>
</table>
Fig 1. Self balancing robot construction and working -- side and front views.


## Description
This repository outlines the implementation of a self balancing robot that uses a kalman filter for angle estimation and an android mobile application for robust control.

<figure>
    <img src="./images/tilt_sbr.png" width="300"/>
    <figcaption>Fig 2. Balancing action of the self balancing robot</figcaption>
</figure>
<br><br>
<figure>
    <img src="./images/app_controller.jpeg" width="300"/>
    <figcaption>Fig 3. Android mobile application control interface.</figcaption>
</figure>

## Getting Started

### Dependencies
* Arduino IDE = 1.8
* Arduino Uno R3 development board

### Installing
```
  git clone https://github.com/Aaatresh/self-balancing-robot
```

### Circuit diagram
<img src="./circuit_diagram/self-balancing-robot-ckt.png" width="1100"/>

### Controlling the robot and executing the program

Construction and working:
* Construct the robot chassis and create a PCB according to the given circuit diagram.
* Compile and load the one of the ```.ino``` files onto an arduino Uno board:
  * ```./code/self_balancing_robot_328P/``` contains code that balances the robot with a single PID controller.
  * ```./code/self_balancing_robot_angle_pos_pid/``` contains code that balances the robot and tracks position set points using two PID controllers.
* Install the ard_bt_controller mobile application onto an android device using its apk file, which can be found in the path: ```./apks/ard_bt_controller.apk```.

## Author
* Anirudh Aatresh ([anirudhashokaatresh@gmail.com](mailto:anirudhashokaatresh@gmail.com))

## License
This project is licensed under the MIT License - see the LICENSE.md file for details.

## Acknowledgments
I would like to thank automaticaddison.com for assistance with the kalman filter implementation.
