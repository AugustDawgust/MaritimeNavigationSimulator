# Autonomous Surface Vessel Navigation Simulator

A Python engineering simulation of an autonomous surface vessel navigating through a two-dimensional environment.

The project is being developed incrementally to demonstrate vessel motion, waypoint navigation, collision avoidance, simplified maneuverability, and mission-performance tracking.

## Current Version: 0.5

Version 0.5 includes:

* Continuous vessel motion in real-world units
* Maritime heading convention
* Frame-rate-independent simulation updates
* Destination-based autonomous navigation
* Shortest-direction heading control
* Realistic maximum turn-rate limiting
* Arrival detection and automatic stopping
* Multiple static circular obstacles
* Conservative circular vessel collision boundary
* Vessel–obstacle collision detection
* Automatic stopping after collision
* Direct-route obstacle detection
* Nearest blocking-obstacle selection
* Automatic temporary avoidance-waypoint generation
* Autonomous navigation around blocking obstacles
* Automatic resumption of destination navigation
* Visible avoidance waypoint and live `AVOIDING` status
* Destination, obstacle, route-trail, and status visualization
* Live heading, speed, distance, and navigation status
* Live mission elapsed-time tracking
* Actual route-distance tracking
* Simplified cubic-speed fuel-consumption model
* Cumulative estimated fuel-use tracking
* Live fuel-burn-rate display
* Automated motion, navigation, obstacle, collision, avoidance, and mission-metric tests
* End-to-end mission-metric validation

## Requirements

* Python 3.13
* Pygame 2.6.1

## Installation

Install the required dependencies:

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

* `main.py` — application entry point and main loop
* `simulation.py` — simulation state, navigation integration, collision handling, mission tracking, and time advancement
* `vessel.py` — vessel state and motion model
* `navigation.py` — heading, turning, and distance calculations
* `obstacle.py` — validated circular obstacle model
* `collision.py` — vessel–obstacle collision calculations
* `avoidance.py` — route-obstruction detection and avoidance-waypoint calculations
* `mission_metrics.py` — fuel calculations and accumulated mission-performance metrics
* `renderer.py` — vessel, destination, obstacle, trail, waypoint, mission-metric, and status visualization
* `config.py` — shared simulation, navigation, fuel-model, and display settings
* `tests/` — automated motion, navigation, obstacle, collision, avoidance, and mission-metric tests
* `assets/` — future visual resources
* `ENGINEERING_NOTES.md` — modeling decisions, assumptions, and limitations

## Development Roadmap

* Version 0.1: continuous vessel motion
* Version 0.2: destination navigation and smooth turning
* Version 0.3: static obstacles and collision detection
* Version 0.4: autonomous obstacle avoidance
* Version 0.5: mission metrics and fuel estimation
* Version 1.0: complete autonomous navigation demonstration
