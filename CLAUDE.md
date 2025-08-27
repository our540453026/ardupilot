# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

ArduPilot is an open-source autopilot software for drones and other unmanned vehicles. It supports multiple vehicle types including multirotors, fixed-wing aircraft, rovers, boats, and submarines.

## Repository Structure

- **ArduCopter/** - Multirotor vehicle code
- **ArduPlane/** - Fixed-wing aircraft code
- **Rover/** - Ground vehicle code
- **ArduSub/** - Underwater vehicle code
- **AntennaTracker/** - Antenna tracking system
- **libraries/** - Shared libraries and components
- **Tools/** - Development tools and utilities

## Build System

ArduPilot uses Waf as its build system. The main build commands are:

```bash
# Configure for a specific board
./waf configure --board <board_name>

# Build a specific vehicle
./waf copter        # Multirotor vehicles
./waf plane         # Fixed-wing aircraft
./waf rover         # Ground vehicles
./waf sub           # Underwater vehicles
./waf antennatracker # Antenna tracker

# List available boards
./waf list_boards

# Clean build
./waf clean
```

Common board names:
- sitl (Software In The Loop simulator)
- CubeBlack
- Pixhawk1
- Pixracer

## Testing

Run tests with:
```bash
# Run relevant tests (only those that need to be run)
./waf check

# Run all tests
./waf check-all
```

## Development Workflow

1. Configure the build for your target board:
   ```bash
   ./waf configure --board sitl
   ```

2. Build the desired vehicle:
   ```bash
   ./waf copter
   ```

3. For SITL development, run the built binary:
   ```bash
   build/sitl/bin/arducopter
   ```

## Key Libraries

- **AP_HAL/** - Hardware Abstraction Layer
- **AP_AHRS/** - Attitude and Heading Reference System
- **AP_GPS/** - GPS handling
- **AP_Compass/** - Compass/magnetometer support
- **AC_AttitudeControl/** - Attitude control algorithms
- **AP_Math/** - Mathematical functions and utilities

## Documentation

- Main documentation: https://ardupilot.org/
- Developer wiki: https://ardupilot.org/dev/
- Build instructions: https://ardupilot.org/dev/docs/building-the-code.html