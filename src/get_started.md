## Introduction

Welcome to the ROSCON FR/DE 2025 "Get Your Hands Dirty" workshop! Thank you for joining us and for your enthusiasm to build, test, and iterate hands-on with ROS 2 and MoveIt 2.

- Date: November 18, 2025
- Location: Strasbourg/Straßburg
- Duration: approximately 4 hours
- Team setup: please form teams of 4 before beginning

Over the next few hours, you and your teammates will explore a ready-to-run robotics stack, integrate motion planning, and iterate on real-world integration tasks. The goal is to learn by doing, share insights, and have fun building together.

Special thanks to the Niryo Core Team : Thomas Deffontaines & Romain Krikorian (Integration Team), Justin Mottier, Thomas Degallaix & Pierre Hantson (Software Team)

Let’s get started!

## Hardware setup

Please ensure the following before we begin:

- Participant equipment (you bring):
  - At least two different computers (laptops) for teamwork and multi-machine ROS 2 networking

- Provided by organizers (we bring):
  - 2x Niryo Ned3 Pro robots
  - 2x conveyor belts with IR sensors
  - 1x Raspberry Pi 4 connected to a screen via HDMI
  - 1x Ethernet switch and 3x Ethernet cables

Connectivity checklist:
- Verify each conveyor belt is connected to the Ned3 Pro back panel (tool connector) and that its IR sensor is connected to the conveyor electronics as specified by Niryo.
- Network 1 (QC line): Raspberry Pi + QC robot + QC conveyor + Computer 1 are connected together via the Ethernet switch.
- Network 2 (Packaging line): Packaging robot + Computer 2 are directly connected via Ethernet (point-to-point).
- Make sure both computers are NOT connected to the same Wi‑Fi network at the same time to avoid cross-talk (ROS 2 is multi-machine by default).
- Alternatively, you can set distinct ROS 2 domains to prevent interference, for instance:
  - Network 1: ROS_DOMAIN_ID=11
  - Network 2: ROS_DOMAIN_ID=22

Network layout overview:

![Workshop network layout](/src/assets/network_layout.svg)

