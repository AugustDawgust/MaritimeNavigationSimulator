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

This makes the simulated speed independent of the rendering frame rate.

## Vessel Model

The vessel is modeled as a point moving at a specified speed and heading. The rendered triangle represents its length, beam, and orientation but does not directly determine its dynamics or collision geometry.

For collision detection, the vessel uses a conservative circular boundary centered on its position. The configured collision radius is large enough to approximately contain the rendered vessel.

## Waypoint Navigation

The vessel autonomously navigates toward one fixed destination.

The desired heading is calculated from the vessel’s current position to the destination using the maritime heading convention. The controller finds the shortest signed heading error so the vessel turns correctly across the 0-degree and 360-degree boundary.

A positive heading error produces a clockwise turn, while a negative heading error produces a counterclockwise turn.

## Turn-Rate Limiting

The vessel cannot change heading instantaneously. Its maximum heading change during one update is:

```text
maximum heading change = maximum turn rate × elapsed time
```

If the remaining heading error is smaller than the permitted change, the vessel turns directly to the desired heading without overshooting it.

## Arrival Detection

The vessel is considered to have arrived when its straight-line distance from the destination is less than or equal to the configured arrival radius.

After arrival:

* The arrival state becomes true.
* Vessel speed is set to zero.
* Navigation and mission-metric updates stop.
* The destination marker and status panel change appearance.

## Obstacle Model

Version 0.3 introduced multiple static circular obstacles.

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

## Autonomous Obstacle Avoidance

Version 0.4 added geometric route planning around static circular obstacles.

Before navigating directly toward the destination, the simulation checks whether any obstacle intersects the required clearance area around the finite route segment. Obstacles behind the vessel or beyond the destination are ignored.

If multiple obstacles block the route, the nearest blocking obstacle is selected first.

The simulator generates a temporary avoidance waypoint perpendicular to the direct route. Its offset from the obstacle center is:

```text
waypoint offset = obstacle radius + avoidance clearance + extra safety offset
```

The vessel then:

1. Navigates toward the temporary waypoint.
2. Clears the waypoint after entering its configured arrival radius.
3. Rechecks the direct route to the final destination.
4. Generates another waypoint if a different obstacle blocks the new route.
5. Resumes destination navigation when the route is clear.

While a temporary waypoint is active, the status panel displays `AVOIDING`.

The current planner always places the waypoint on one predetermined side of the route. It does not yet compare alternative routes, optimize travel distance, account for moving obstacles, or guarantee a solution in tightly clustered obstacle fields.

## Mission Metrics

Version 0.5 adds live tracking of:

* Mission elapsed time
* Actual route distance traveled
* Estimated fuel used
* Current estimated fuel-burn rate

Elapsed time accumulates during active simulation updates. Once the vessel arrives or collides, future updates stop and the recorded mission metrics remain unchanged.

Route distance is calculated after each movement update from the vessel’s previous and current positions:

```text
distance traveled = √[(x₂ − x₁)² + (y₂ − y₁)²]
```

The calculated distance is added to the cumulative route distance. This measures the vessel’s actual simulated path, including turns and obstacle-avoidance detours, rather than only measuring the straight-line distance between the starting point and destination.

## Fuel-Consumption Model

Version 0.5 uses a simplified cubic-speed model to estimate fuel consumption while the vessel is moving:

```text
fuel burn rate = base fuel burn rate + speed coefficient × speed³
```

The cubic term approximates the general marine-engineering relationship in which the propulsion power required by a displacement vessel increases approximately with the cube of speed under comparable operating conditions.

The fuel used during one update is:

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
* The destination marker
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
* Navigation and avoidance status

Trail points are stored only after the vessel moves a configured minimum distance from the previous point. This avoids storing a new trail point every frame.

## Current Limitations

* One fixed destination
* Constant speed until arrival or collision
* Fixed maximum turn rate
* Point-based vessel motion
* Circular approximation of vessel collision geometry
* Discrete collision checks
* Static obstacles only
* No acceleration or deceleration model
* No wind, waves, or current
* Fixed camera
* Avoidance uses one temporary waypoint at a time
* Avoidance always selects one predetermined side of an obstacle
* No path optimization or alternative-route comparison
* No support for moving obstacles
* No guarantee of a valid route through tightly clustered obstacles
* Simplified cubic-speed fuel model
* Fuel coefficients are not calibrated to a real vessel
* No engine-efficiency or propeller-efficiency model
* No engine transients, idle consumption, or auxiliary electrical loads
* No fuel-tank capacity or fuel-depletion behavior
* No effects from vessel loading, hull condition, water depth, or sea state
