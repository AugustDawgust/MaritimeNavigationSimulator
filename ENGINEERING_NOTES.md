# Engineering Notes

## Coordinate System

The simulation uses real-world units:

- Position is measured in meters.
- Speed is measured in meters per second.
- The positive x-axis points east.
- The positive y-axis points north.
- Heading is measured clockwise from north.

This convention resembles maritime navigation rather than Pygame's native screen-coordinate system.

## Time Integration

Vessel position is updated using the actual elapsed time between frames:

position change = velocity × elapsed time

This makes the simulated speed independent of the rendering frame rate.

## Vessel Model

The vessel is currently modeled as a point moving at a specified speed and heading. The rendered triangle represents its length, beam, and orientation but does not yet affect its dynamics or collision geometry.

## Waypoint Navigation

Version 0.2 adds autonomous navigation toward one fixed destination.

The desired heading is calculated from the vessel's current position to the destination using the maritime heading convention. The controller finds the shortest signed heading error so it turns correctly across the 0-degree and 360-degree boundary.

A positive heading error produces a clockwise turn, while a negative error produces a counterclockwise turn.

## Turn-Rate Limiting

The vessel cannot change heading instantaneously. Its maximum heading change during one update is:

maximum heading change = maximum turn rate × elapsed time

If the remaining heading error is smaller than the permitted change, the vessel turns directly to the desired heading without overshooting it.

## Arrival Detection

The vessel is considered to have arrived when its straight-line distance from the destination is less than or equal to the configured arrival radius.

After arrival:

- The arrival state becomes true.
- Vessel speed is set to zero.
- Navigation updates stop.
- The destination marker and status panel change appearance.

## Rendering

The physics model stores position in meters. The renderer separately converts meters into pixels using a fixed display scale.

Positive world y-coordinates point north, while Pygame screen y-coordinates increase downward. The renderer reverses the y-direction during conversion.

The renderer also displays:

- The vessel
- The destination marker
- A sampled route trail
- Current heading
- Current speed
- Distance remaining
- Navigation status

Trail points are stored only after the vessel moves a configured minimum distance from the previous point. This avoids storing a new trail point every frame.

## Current Limitations

- One fixed destination
- Constant speed until arrival
- Fixed maximum turn rate
- Point-based vessel dynamics
- No acceleration or deceleration model
- No wind, waves, or current
- No obstacles or collision detection
- No autonomous obstacle avoidance
- Fixed camera