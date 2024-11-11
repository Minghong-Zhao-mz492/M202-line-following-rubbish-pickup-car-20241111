# Turtle Simulation for Line-Following and Rubbish Sorting

This simulation uses Python's `turtle` module to create a virtual environment where a robotic car (represented by a yellow turtle) navigates a predefined path, collects "rubbish," and places it in designated zones based on color.

## Purpose

The purpose of this simulation is to visually demonstrate how a robotic car can follow a set path, identify and collect rubbish, and deposit it in specific areas marked by colors. This is a virtual model of a real-world task of sorting rubbish by color.

## Features

- **Path Layout**: A predefined path is drawn on the screen to simulate lanes and boundaries for the car's movement.
- **Rubbish Generation**: Rubbish objects are represented as circles in random colors (green or red) and placed at specific locations along the path.
- **Sorting Zones**: Designated zones for rubbish sorting are created—green and red areas representing different sorting destinations.
- **Robotic Car**: A yellow turtle represents the car, which navigates the path, collects rubbish, and drops it in the correct zone.

## Code Overview

- **Screen Setup**: Initializes a black background and sets up the screen dimensions.
- **Lane Drawing**: Defines and draws static path elements using a helper function `draw_line()` to simulate lanes and boundaries.
- **Rubbish Placement**: Creates rubbish circles in random colors (green or red) and places them at specific points along the path.
- **Sorting Zones**: Draws red and green zones where the car deposits rubbish based on its color.
- **Robotic Car Movement**: The yellow turtle car moves along the path, checks the color of each rubbish object, and performs actions based on the color:
  - If the rubbish is green, it moves to the green zone.
  - If the rubbish is red, it moves to the red zone.

## Key Functions

- `draw_line(x_start, y_start, x_end, y_end)`: Draws lanes on the path by specifying start and end coordinates.
- `draw_square(x_start, y_start, width, height, color)`: Draws sorting zones (red and green) at specified locations.
- `onetog()` and `twotog()`: Functions that direct the car to pick up and place green rubbish in the green zone.
- `onetor()` and `twotor()`: Functions that direct the car to pick up and place red rubbish in the red zone.

## How It Works

1. **Initialization**: The screen and path layout are set up, and rubbish objects are created at specific positions with random colors (either green or red).
2. **Sorting Process**: The car navigates along the path, detecting rubbish objects and determining their color.
3. **Rubbish Collection and Sorting**: Depending on the color, the car transports each piece of rubbish to the corresponding zone (green or red).

This simulation showcases the fundamental logic of a line-following rubbish-sorting robot in a virtual environment using the `turtle` module. It serves as a model for building similar real-world robotic systems.
