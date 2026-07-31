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

Version 0.1 models the vessel as a point moving at constant speed and heading. The rendered triangle represents its length, beam, and orientation but does not yet affect its dynamics.

## Rendering

The physics model stores position in meters. The renderer separately converts meters into pixels using a fixed display scale.

Positive world y-coordinates point north, while Pygame screen y-coordinates increase downward. The renderer reverses the y-direction during conversion.

## Current Limitations

- Constant speed and heading
- No destination or navigation controller
- No turn-rate limit
- No acceleration or deceleration
- No obstacles or collision detection
- Fixed camera