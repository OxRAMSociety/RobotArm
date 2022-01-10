![logo](https://user-images.githubusercontent.com/6771224/141656663-96ae20bf-9eaf-4808-b145-5f7c027ac39d.png)

<h1 align="center">&middot; RobotArm &middot;</h1>


<p align="center">
Teaching a robotic arm how to interact with the world.
<br/>
<a href="https://github.com/OxRAMSociety/RobotArm/projects/1">Roadmap</a>
&middot;
<a href="https://github.com/OxRAMSociety/RobotArm/wiki">Wiki</a>
&middot;
<a href="https://www.oxramsociety.org/blog/category/oxarm">Blog</a>
<br/><br/>
<a href="https://github.com/OxRAMSociety/RobotArm/issues">
    <img src="https://img.shields.io/github/issues/OxRAMSociety/RobotArm.svg?label=TASKS&style=for-the-badge" alt="Tasks badge">
</a>
<a href="LICENSE">
    <img src="https://img.shields.io/github/license/OxRAMSociety/RobotArm.svg?style=for-the-badge" alt="License badge">
</a>
<a href="mailto:project.robotarm@oxramsociety.org">
    <img src="https://img.shields.io/badge/Contact_Us-0078D4?style=for-the-badge&logo=maildotru" alt="Email">
</a>
<br/><br/>
<img alt="Twitter Follow button" src="https://img.shields.io/twitter/follow/ox3dp?style=social">
</p>

# About

This is a public repository gathering all the code and resources related to the OxRAM Robot Arm project.

The goal of the project is to build a robotic arm using free and open source resources, including software, electronics and 3D printed design. We are using [RBX1] as the initial arm design and [ROS] as the software stack to program and interact with the arm.

One of our main goals is to *provide the arm with a camera* to give it a certain degree of autonomy via image processing and machine learning techniques. As a first *toy application*, the arm will be able to *play chess*, being able to recognize and interact with a physical board and its chess pieces.

> **NOTE** that this is a highly WIP project and works on the hardware side have been slowed down by the pandemic and the inability to access our workshop.
> We are in the process of resuming the works at the moment.

## Features

- The arm design is based on [RBX1], a 6DOF 3D printed robotic arm
- The electronics are based on the [BCN3D MOVEO] arm
    - The schematics are being reworked to manage the additional degree of freedom
    - A combination of [Arduino Mega 2560] and [RAMPS 1.4 shield] is used in place of the custom [SlushEngine Model D 7] stepper motor driver to minimise costs
- [ROS] is used to power the software side of the arm and to provide a *simulation environment*
- Ability to understand (domain-specific) input voice commands (English and Chinese)
- Ability to perform (domain-specific) object detection and recognition through a camera mounted on the gripper

## Acknowledgements

- [RBX1] and [BCN3D MOVEO] 3D arm designs
- [ROS] software stack
<!-- - Blender -->
<!-- - software used for voice rec? -->
<!-- - software used for training image rec? -->

## Sponsors

`todo`

## License

The code in this repository is licensed under the [MIT License](LICENSE), unless otherwise stated.

Any other resources (e.g., documentation, media) are licensed under the [Creative Commons Attribution-ShareAlike 4.0 International License](http://creativecommons.org/licenses/by-sa/4.0/), unless otherwise stated.

<!-- Resources -->

[RBX1]: https://roboteurs.com/products/rbx1-remix-3d-printed-6-axis-robot-arm-kit
[BCN3D MOVEO]: https://www.bcn3d.com/bcn3d-moveo-the-future-of-learning/
[ROS]: https://www.ros.org/
[Arduino Mega 2560]: https://store.arduino.cc/arduino-mega-2560-rev3
[RAMPS 1.4 shield]: https://reprap.org/wiki/RAMPS_1.4
[SlushEngine Model D 7]: https://roboteurs.com/products/slushengine-model-d
