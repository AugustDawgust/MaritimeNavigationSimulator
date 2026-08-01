# Autonomous Surface Vessel Navigation Simulator

A Python engineering simulation of an autonomous surface vessel navigating through a two-dimensional environment.

The project is being developed incrementally to demonstrate vessel motion, waypoint navigation, collision avoidance, simplified maneuverability, safe navigation failure, interactive mission control, and mission-performance tracking.

## Current Version: 0.9

Version 0.9 includes:

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
* Left-click destination selection
* Interactive mission resetting from the vessel’s current position
* Automatic recovery from arrival, collision, and blocked-navigation states when a new destination is selected
* Mission-metric and route-trail resetting for each new destination
* Screen-to-world coordinate conversion for mouse input
* Visible avoidance waypoint
* Live `NAVIGATING`, `AVOIDING`, `ARRIVED`, `COLLISION`, and `NAVIGATION BLOCKED` status
* Destination, obstacle, route-trail, and status visualization
* Live heading, speed, and distance display
* Live mission elapsed-time tracking
* Actual route-distance tracking
* Simplified cubic-speed fuel-consumption model
* Cumulative estimated fuel-use tracking
* Live fuel-burn-rate display
* Automated motion, navigation, obstacle, collision, avoidance, safe-failure, coordinate-conversion, mission-reset, and mission-metric tests
* End-to-end navigation and mission-metric validation
* 58 passing automated tests

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

## Interactive Controls

Left-click anywhere in the simulation window to select a new destination.

Selecting a destination:

* Moves the destination marker to the selected location
* Starts a new mission from the vessel’s current position
* Preserves the vessel’s current heading
* Restores the vessel’s configured operating speed
* Clears any arrival, collision, or blocked-navigation state
* Clears the active avoidance waypoint
* Restarts the route trail at the vessel’s current position
* Resets elapsed time, route distance, and estimated fuel use
* Preserves the existing obstacle layout

The vessel then redirects toward the new destination using its normal heading controller and turn-rate limit.

## Running the Tests

```bash
python -m unittest discover -s tests
```

The complete Version 0.9 suite should report:

```text
Ran 58 tests
OK
```

## Project Structure

* `main.py` — application entry point, event loop, mouse-input handling, simulation updates, and renderer integration
* `simulation.py` — simulation state, destination resetting, active-target selection, navigation integration, terminal-state handling, mission tracking, and time advancement
* `vessel.py` — vessel state and point-based motion model
* `navigation.py` — heading, turning, and distance calculations
* `obstacle.py` — validated circular obstacle model
* `collision.py` — vessel–obstacle collision calculations
* `avoidance.py` — route-obstruction detection, blocking-obstacle selection, two-sided waypoint generation, and safe-candidate selection
* `mission_metrics.py` — fuel calculations and accumulated mission-performance metrics
* `renderer.py` — coordinate conversion and vessel, destination, obstacle, trail, waypoint, mission-metric, and navigation-status visualization
* `config.py` — shared simulation, navigation, avoidance, fuel-model, status-color, and display settings
* `tests/` — automated motion, navigation, obstacle, collision, avoidance, safe-failure, mission-reset, coordinate-conversion, and mission-metric tests
* `assets/` — future visual resources
* `ENGINEERING_NOTES.md` — modeling decisions, algorithms, assumptions, validation, and limitations

## Navigation Behavior

The vessel normally steers toward the selected final destination.

When an obstacle blocks the finite direct route, the controller:

1. Selects the nearest blocking obstacle.
2. Generates a preferred temporary waypoint on one side.
3. Generates a mirrored waypoint on the opposite side.
4. Checks whether each waypoint can be approached with the configured obstacle clearance.
5. Selects the preferred waypoint when safe.
6. Falls back to the opposite waypoint when necessary.
7. Stops with `NAVIGATION BLOCKED` if neither waypoint is safe.
8. Rechecks the final-destination route after completing each avoidance maneuver.

This reactive system can make separate avoidance decisions for multiple obstacles without requiring a global route planner.

## Interactive Mission Reset

Selecting a new destination begins a fresh mission without resetting the vessel’s physical pose or the environment.

The reset process:

1. Stores the newly selected destination.
2. Clears arrival, collision, and blocked-navigation states.
3. Clears any temporary avoidance waypoint.
4. Restores the configured vessel speed.
5. Restarts the trail at the vessel’s current position.
6. Creates a new mission-metrics record.
7. Preserves the vessel’s position and heading.
8. Preserves all existing obstacles.

This allows the simulator to recover interactively from any terminal state and supports repeated navigation demonstrations during one application session.

## Mission Metrics

During an active mission, the simulator tracks:

* Elapsed time
* Actual route distance
* Estimated cumulative fuel use
* Current estimated fuel-burn rate

Mission metrics stop updating after arrival, collision, or blocked navigation. They reset when the user selects a new destination.

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
* Version 0.9: interactive destination selection and mission resetting
* Version 1.0: complete autonomous navigation demonstration

## Current Limitations

* Constant operating speed during each active mission
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
* No guarantee of a valid route through tightly clustered obstacle fields
* New destinations are not validated before a mission begins
* A destination can be selected inside or near an obstacle
* Mouse input is the only interactive mission control
* Simplified, uncalibrated cubic-speed fuel model
* No engine, propeller, loading, water-depth, or sea-state effects