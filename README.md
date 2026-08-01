# Autonomous Surface Vessel Navigation Simulator

A Python engineering simulation of an autonomous surface vessel navigating through a two-dimensional environment.

The project is being developed incrementally to demonstrate vessel motion, waypoint navigation, collision avoidance, simplified maneuverability, safe navigation failure, and mission-performance tracking.

## Current Version: 0.8

Version 0.8 includes:

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
* Preferred-side and opposite-side avoidance candidates
* Candidate safety checks against all obstacles
* Automatic fallback to the opposite side when the preferred side is unsafe
* Safe stopping when neither avoidance side is available
* Dedicated `navigation_blocked` simulation state
* Automatic resumption of destination navigation after successful avoidance
* Validated navigation around aligned and staggered multiple-obstacle layouts
* Visible avoidance waypoint
* Live `NAVIGATING`, `AVOIDING`, `ARRIVED`, `COLLISION`, and `NAVIGATION BLOCKED` status
* Destination, obstacle, route-trail, and status visualization
* Live heading, speed, and distance display
* Live mission elapsed-time tracking
* Actual route-distance tracking
* Simplified cubic-speed fuel-consumption model
* Cumulative estimated fuel-use tracking
* Live fuel-burn-rate display
* Automated motion, navigation, obstacle, collision, avoidance, safe-failure, and mission-metric tests
* End-to-end navigation and mission-metric validation
* 56 passing automated tests

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

The complete Version 0.8 suite should report:

```text
Ran 56 tests
OK
```

## Project Structure

* `main.py` — application entry point, event loop, simulation updates, and renderer integration
* `simulation.py` — simulation state, active-target selection, navigation integration, terminal-state handling, mission tracking, and time advancement
* `vessel.py` — vessel state and point-based motion model
* `navigation.py` — heading, turning, and distance calculations
* `obstacle.py` — validated circular obstacle model
* `collision.py` — vessel–obstacle collision calculations
* `avoidance.py` — route-obstruction detection, blocking-obstacle selection, two-sided waypoint generation, and safe-candidate selection
* `mission_metrics.py` — fuel calculations and accumulated mission-performance metrics
* `renderer.py` — vessel, destination, obstacle, trail, waypoint, mission-metric, and navigation-status visualization
* `config.py` — shared simulation, navigation, avoidance, fuel-model, status-color, and display settings
* `tests/` — automated motion, navigation, obstacle, collision, avoidance, safe-failure, and mission-metric tests
* `assets/` — future visual resources
* `ENGINEERING_NOTES.md` — modeling decisions, algorithms, assumptions, validation, and limitations

## Navigation Behavior

The vessel normally steers toward the final destination.

When an obstacle blocks the finite direct route, the controller:

1. Selects the nearest blocking obstacle.
2. Generates a preferred temporary waypoint on one side.
3. Generates a mirrored waypoint on the opposite side.
4. Checks whether each waypoint can be approached with the configured obstacle clearance.
5. Selects the preferred waypoint when safe.
6. Falls back to the opposite waypoint when necessary.
7. Stops with `NAVIGATION BLOCKED` if neither waypoint is safe.
8. Rechecks the final destination route after completing each avoidance maneuver.

This reactive system can make separate avoidance decisions for multiple obstacles without requiring a global route planner.

## Mission Metrics

During an active mission, the simulator tracks:

* Elapsed time
* Actual route distance
* Estimated cumulative fuel use
* Current estimated fuel-burn rate

Mission metrics stop updating after arrival, collision, or blocked navigation.

Fuel consumption uses a simplified cubic-speed relationship intended for engineering comparison rather than real-vessel prediction.

## Development History and Roadmap

* Version 0.1: continuous vessel motion
* Version 0.2: destination navigation and smooth turning
* Version 0.3: static obstacles and collision detection
* Version 0.4: autonomous obstacle avoidance
* Version 0.5: mission metrics and fuel estimation
* Version 0.6: expanded navigation reliability and end-to-end avoidance validation
* Version 0.7: validated multiple aligned and staggered static obstacles
* Version 0.8: two-sided safe waypoint selection and blocked-navigation failure handling
* Version 0.9: planned interactive destination selection and mission reset
* Version 1.0: complete autonomous navigation demonstration

## Current Limitations

* One fixed destination
* Constant speed until arrival, collision, or blocked navigation
* Fixed maximum turn rate
* Point-based vessel motion
* Circular approximation of vessel collision geometry
* Discrete collision checks
* Static obstacles only
* No acceleration or deceleration model
* No wind, waves, or current
* Fixed camera
* One temporary avoidance waypoint at a time
* Preferred safe side is selected rather than globally optimized
* No complete-route path optimization
* No support for moving obstacles
* No guarantee of a valid route through tightly clustered obstacles
* Simplified, uncalibrated cubic-speed fuel model
* No engine, propeller, loading, water-depth, or sea-state effects