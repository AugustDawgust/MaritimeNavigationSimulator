# Autonomous Surface Vessel Navigation Simulator

A Python engineering simulation of an autonomous surface vessel navigating through a two-dimensional environment.

The project is being developed incrementally to demonstrate vessel motion, waypoint navigation, collision avoidance, and simplified maneuverability.

## Current Version: 0.2

Version 0.2 includes:

- Continuous vessel motion in real-world units
- Maritime heading convention
- Frame-rate-independent simulation updates
- Destination-based autonomous navigation
- Shortest-direction heading control
- Realistic maximum turn-rate limiting
- Arrival detection and automatic stopping
- Destination and route-trail visualization
- Live heading, speed, distance, and navigation status
- Automated motion and navigation tests

## Requirements

- Python 3.13
- Pygame 2.6.1

## Installation

Install the required package:

```bash
python -m pip install -r requirements.txt
```

## Running the Simulator

```bash
python main.py
```

## Running the Tests

```bash
python -m unittest discover -s tests
```

## Project Structure

- `main.py` — application entry point and main loop
- `simulation.py` — simulation state, navigation integration, and time advancement
- `vessel.py` — vessel state and motion model
- `navigation.py` — heading, turning, and distance calculations
- `renderer.py` — simulation visualization and status display
- `config.py` — shared simulation and display settings
- `tests/` — automated motion and navigation tests
- `assets/` — future visual resources
- `ENGINEERING_NOTES.md` — modeling decisions, assumptions, and limitations

## Development Roadmap

- Version 0.1: continuous vessel motion
- Version 0.2: destination navigation and smooth turning
- Version 0.3: static obstacles and collision detection
- Version 0.4: autonomous obstacle avoidance
- Version 0.5: mission metrics and fuel estimation
- Version 1.0: complete autonomous navigation demonstration