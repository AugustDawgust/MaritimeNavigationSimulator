# Engineering Notes

## Coordinate System

The simulation uses real-world units:

* Position is measured in meters.
* Speed is measured in meters per second.
* The positive x-axis points east.
* The positive y-axis points north.
* Heading is measured clockwise from north.

This convention resembles maritime navigation rather than Pygame’s native screen-coordinate system.

## Time Integration

Vessel position is updated using the actual elapsed time between frames:

```text
position change = velocity × elapsed time
```

This makes simulated speed independent of the rendering frame rate.

## Vessel Model

The vessel is modeled as a point moving at a specified speed and heading. The rendered triangle represents its length, beam, and orientation but does not directly determine its dynamics or collision geometry.

For collision detection, the vessel uses a conservative circular boundary centered on its position. The configured collision radius is large enough to approximately contain the rendered vessel.

## Waypoint Navigation

The vessel autonomously navigates toward the currently selected final destination.

The desired heading is calculated from the vessel’s current position to the active target using the maritime heading convention. The active target is either:

* The final destination
* A temporary obstacle-avoidance waypoint

The controller finds the shortest signed heading error so the vessel turns correctly across the 0-degree and 360-degree boundary.

A positive heading error produces a clockwise turn, while a negative heading error produces a counterclockwise turn.

## Turn-Rate Limiting

The vessel cannot change heading instantaneously. Its maximum heading change during one update is:

```text
maximum heading change = maximum turn rate × elapsed time
```

If the remaining heading error is smaller than the permitted change, the vessel turns directly to the desired heading without overshooting it.

## Arrival Detection

The vessel is considered to have arrived when its straight-line distance from the final destination is less than or equal to the configured arrival radius.

After arrival:

* The arrival state becomes true.
* Vessel speed is set to zero.
* Navigation and mission-metric updates stop.
* The destination marker and status panel change appearance.
* The status panel displays `ARRIVED`.

A new destination can be selected after arrival to clear the arrival state and begin another mission.

## Obstacle Model

The environment contains multiple static circular obstacles.

Each obstacle stores:

* An x-coordinate in meters
* A y-coordinate in meters
* A radius in meters

Obstacle radii must be greater than zero. Attempting to create an obstacle with a zero or negative radius raises a `ValueError`.

## Collision Detection

Both the vessel collision boundary and each obstacle are treated as circles.

A collision occurs when:

```text
center distance ≤ vessel collision radius + obstacle radius
```

Touching boundaries therefore count as a collision.

The simulation checks the vessel against every obstacle after each movement update. If any collision is detected:

* The collision state becomes true.
* Vessel speed is set to zero.
* Future simulation and mission-metric updates stop.
* The status panel displays `COLLISION`.

A new destination can be selected after collision to clear the collision state and begin another mission from the vessel’s current position.

Because collision detection occurs at discrete time steps, extremely large elapsed-time values could allow the vessel to move through an obstacle between checks. Normal frame times make this unlikely, but continuous collision detection could address this limitation in a future version.

## Direct-Route Obstacle Detection

Before navigating directly toward the final destination, the simulation checks whether any obstacle intersects the configured clearance area around the finite route segment.

Obstacles behind the vessel or beyond the destination are ignored.

If multiple obstacles block the route, the nearest blocking obstacle is selected first. Distance is measured from the vessel’s current position to each blocking obstacle’s center.

After completing one avoidance maneuver, the vessel rechecks the direct route. This allows the reactive controller to make separate avoidance decisions for multiple aligned or staggered obstacles.

## Autonomous Obstacle Avoidance

The simulator uses geometric temporary-waypoint navigation around static circular obstacles.

For the nearest blocking obstacle, the controller first generates a preferred avoidance waypoint perpendicular to the direct route. Its offset from the obstacle center is:

```text
waypoint offset =
    obstacle radius
    + avoidance clearance
    + extra safety offset
```

The opposite-side candidate is created by reflecting the preferred waypoint across the obstacle’s center.

The controller evaluates the two candidates in this order:

1. Preferred-side waypoint
2. Opposite-side waypoint

A candidate is considered unsafe if the finite approach segment from the vessel to that waypoint intersects the configured clearance area around any obstacle.

The controller selects the preferred candidate when it is safe. If the preferred candidate is unsafe, it uses the opposite candidate when available.

The vessel then:

1. Navigates toward the selected temporary waypoint.
2. Clears the waypoint after entering its configured arrival radius.
3. Rechecks the direct route to the final destination.
4. Generates another waypoint if an obstacle still blocks the route.
5. Resumes destination navigation when the route is clear.

The global route planner constructs a complete collision-free route before navigation begins. It uses obstacle-clearance geometry and selects a route from the vessel’s current position to the destination. The vessel then follows the resulting waypoint sequence in order.

If no safe route can be found, the vessel stops and enters the `NAVIGATION BLOCKED` state.

This reactive process has been validated with both aligned and staggered multiple-obstacle scenarios.

## Safe Navigation Failure

If neither avoidance-side candidate provides a safe approach, the simulator enters a blocked-navigation state instead of selecting an unsafe target.

When navigation becomes blocked:

* `navigation_blocked` becomes true.
* No avoidance waypoint is activated.
* Vessel speed is set to zero.
* The final destination remains unchanged.
* Future simulation and mission-metric updates stop.
* The status panel displays `NAVIGATION BLOCKED`.

The blocked state is distinct from a collision. The vessel stops before knowingly committing to either unsafe candidate.

Selecting a new destination clears the blocked state and begins a new mission from the vessel’s current position.

## Terminal States

The simulation has three terminal states:

* Arrival
* Collision
* Navigation blocked

Once any terminal state becomes active, future motion and mission-metric updates stop.

The status panel prioritizes terminal and navigation states in this order:

1. `COLLISION`
2. `ARRIVED`
3. `NAVIGATION BLOCKED`
5. `NAVIGATING`

All three terminal states can be cleared by selecting a new destination.

## Interactive Destination Selection

Version 0.9 introduced interactive destination selection using the left mouse button.

When a left-click event occurs, the application:

1. Reads the cursor’s screen position.
2. Converts that position from pixels to world coordinates.
3. Passes the resulting coordinates to `Simulation.set_destination()`.
4. Begins a fresh mission toward the selected location.

This allows destinations to be changed during an active mission or after arrival, collision, or blocked navigation.

## Screen-to-World Coordinate Conversion

Pygame reports cursor positions in screen coordinates:

* The screen origin is at the upper-left corner.
* Positive screen x points right.
* Positive screen y points downward.

The simulation uses world coordinates centered in the window:

* Positive world x points east.
* Positive world y points north.

Screen x is converted to world x using:

```text
world x =
    (screen x − window width / 2)
    / pixels per meter
```

Screen y is converted to world y using:

```text
world y =
    (window height / 2 − screen y)
    / pixels per meter
```

The reversed subtraction in the y conversion accounts for the opposite vertical-axis directions used by the simulation and Pygame.

This operation is the inverse of the renderer’s world-to-screen coordinate conversion.

## Mission Reset Behavior

`Simulation.set_destination()` begins a new mission without recreating the entire simulation or moving the vessel back to its original starting point.

When a new destination is selected:

* The destination coordinates are replaced.
* `arrived` becomes false.
* `collided` becomes false.
* `navigation_blocked` becomes false.
* The active avoidance waypoint is cleared.
* Vessel speed is restored to the configured initial operating speed.
* The route trail is replaced with one point at the vessel’s current position.
* Mission elapsed time is reset to zero.
* Route distance is reset to zero.
* Estimated fuel use is reset to zero.
* The vessel’s position is preserved.
* The vessel’s heading is preserved.
* The obstacle collection is preserved.

Preserving position and heading means the new mission begins from the vessel’s actual current pose. The normal turn-rate-limited controller then redirects the vessel toward the new destination.

This differs from a complete application restart, which would recreate the vessel at its configured initial position and heading.

## Mission Metrics

The simulation tracks:

* Mission elapsed time
* Actual route distance traveled
* Estimated fuel used
* Current estimated fuel-burn rate

Elapsed time accumulates during active simulation updates. Once the vessel arrives, collides, or becomes navigation blocked, future updates stop and the recorded mission metrics remain unchanged.

Selecting a new destination creates a fresh mission-metrics record with zero elapsed time, distance, and fuel use.

Route distance is calculated after each movement update from the vessel’s previous and current positions:

```text
distance traveled = √[(x₂ − x₁)² + (y₂ − y₁)²]
```

The calculated distance is added to the cumulative route distance. This measures the vessel’s actual simulated path, including turns and obstacle-avoidance detours, rather than only measuring the straight-line distance between the starting point and destination.

## Fuel-Consumption Model

The simulator uses a simplified cubic-speed model to estimate fuel consumption while the vessel is moving:

```text
fuel burn rate = base fuel burn rate + speed coefficient × speed³
```

The cubic term approximates the general marine-engineering relationship in which the propulsion power required by a displacement vessel increases approximately with the cube of speed under comparable operating conditions.

Fuel used during one update is:

```text
fuel used = fuel burn rate × elapsed time / 3600
```

The division by 3,600 converts elapsed time from seconds to hours because the burn rate is measured in liters per hour.

When vessel speed is zero, the model returns a fuel-burn rate of zero. The configured base rate therefore represents an underway fuel-consumption component rather than engine idling or hotel loads.

Fuel consumption is accumulated using the vessel’s speed during each movement interval. At the current constant operating speed, the resulting mission estimate is proportional to total travel time.

The model is intended for simulation and comparison purposes. Its coefficients are not calibrated to a specific engine, hull, propeller, vessel displacement, sea condition, or measured fuel-consumption curve.

## Rendering

The physics model stores position in meters. The renderer separately converts meters into pixels using a fixed display scale.

Positive world y-coordinates point north, while Pygame screen y-coordinates increase downward. The renderer reverses the y-direction during conversion.

The renderer displays:

* The vessel
* The selected final-destination marker
* Multiple circular obstacles
* A sampled route trail
* The active avoidance waypoint
* Current heading
* Current speed
* Distance remaining
* Mission elapsed time
* Actual route distance
* Estimated cumulative fuel use
* Current estimated fuel-burn rate
* Navigation, avoidance, arrival, collision, and blocked status

Trail points are stored only after the vessel moves a configured minimum distance from the previous point. This avoids storing a new trail point every frame.

When a new destination is selected, the previous trail is cleared and a new trail begins at the vessel’s current position.

## Automated Validation

The Version 0.9 test suite contains 58 automated tests covering:

* Vessel motion
* Frame-rate-independent updates
* Maritime heading calculations
* Shortest-direction turning
* Turn-rate limiting
* Arrival behavior
* Obstacle validation
* Circular collision detection
* Direct-route obstruction detection
* Nearest blocking-obstacle selection
* Avoidance-waypoint calculations
* End-to-end obstacle avoidance
* Multiple aligned obstacles
* Multiple staggered obstacles
* Preferred-side and opposite-side selection
* Safe stopping when both avoidance sides are blocked
* Interactive mission-state resetting
* Preservation of vessel position and heading during mission reset
* Preservation of existing obstacles during mission reset
* Restoration of vessel speed
* Trail and mission-metric resetting
* Screen-to-world coordinate conversion
* Mission elapsed time
* Actual route distance
* Fuel-burn calculations
* Cumulative fuel use
* Terminal-state metric freezing

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
* Avoidance uses one temporary waypoint at a time
* Preferred-side candidate is checked before the opposite side
* No comparison of total route length between safe candidates
* Candidate safety considers the approach to the temporary waypoint rather than planning the complete remaining route
* No global path optimization
* No support for moving obstacles
* No guarantee of a valid route through tightly clustered obstacle fields
* New destinations are not validated before the mission begins
* A destination can be selected inside an obstacle or inside its required clearance area
* Selecting a new destination after collision clears the collision state without repositioning the vessel
* Mouse input is the only interactive mission control
* Simplified cubic-speed fuel model
* Fuel coefficients are not calibrated to a real vessel
* No engine-efficiency or propeller-efficiency model
* No engine transients, idle consumption, or auxiliary electrical loads
* No fuel-tank capacity or fuel-depletion behavior
* No effects from vessel loading, hull condition, water depth, or sea state