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

The vessel autonomously navigates toward one fixed final destination.

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

While a temporary waypoint is active, the status panel displays `AVOIDING`.

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
4. `AVOIDING`
5. `NAVIGATING`

## Mission Metrics

The simulation tracks:

* Mission elapsed time
* Actual route distance traveled
* Estimated fuel used
* Current estimated fuel-burn rate

Elapsed time accumulates during active simulation updates. Once the vessel arrives, collides, or becomes navigation blocked, future updates stop and the recorded mission metrics remain unchanged.

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
* The final destination marker
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

## Automated Validation

The test suite contains 56 automated tests covering:

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
* Mission elapsed time
* Actual route distance
* Fuel-burn calculations
* Cumulative fuel use
* Terminal-state metric freezing

## Current Limitations

* One fixed final destination
* Constant speed until arrival, collision, or blocked navigation
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
* Blocked navigation requires an external reset to resume
* Simplified cubic-speed fuel model
* Fuel coefficients are not calibrated to a real vessel
* No engine-efficiency or propeller-efficiency model
* No engine transients, idle consumption, or auxiliary electrical loads
* No fuel-tank capacity or fuel-depletion behavior
* No effects from vessel loading, hull condition, water depth, or sea state