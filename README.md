# Broken Wheels: Obstacle Course Robot

![Arduino](https://img.shields.io/badge/Arduino-00878F?logo=arduino&logoColor=white)
![C++](https://img.shields.io/badge/C%2B%2B-00599C?logo=cplusplus&logoColor=white)
![UTRAHacks](https://img.shields.io/badge/UTRAHacks-Communication%20Award-E7352C)

> An Arduino-powered autonomous robot that races through a line-following obstacle course as fast as possible.

Built at **UTRAHacks**, Canada's biggest robotics hackathon, where our team (**Broken Wheels, Team 39**) won the **Communication Award**.

## Table of Contents

- [Features](#features)
- [Hardware](#hardware)
- [How It Works](#how-it-works)
- [Flashing](#flashing)

## Features

- PID line-following tuned for fast, accurate navigation.
- Color-sensor detection of the start and finish markers.
- Optimized to complete the winding course in the shortest time while staying on the line.

## Hardware

- Arduino R4 UNO Minima
- Two DC motors driven through an H-bridge
- IR line sensors
- A color sensor for start and finish detection

## How It Works

The robot reads line position from its IR sensors and feeds the error into a PID controller that adjusts the two motor speeds for smooth, high-speed tracking. A color sensor detects the start and finish marker to time and end the run. All pin assignments and tuning constants are documented at the top of the sketch.

## Flashing

1. Open `obstacle_course_FINAL_CLEAN.ino` in the Arduino IDE.
2. Select the **Arduino R4 UNO Minima** board.
3. Connect the board and click **Upload**.
