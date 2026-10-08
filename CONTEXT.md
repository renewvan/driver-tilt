# node-tilt

Domain glossary for node-tilt: what each published field *means*,
independent of the Python that computes it. Entity-level vocabulary
(**Tilt**) lives in `hub/CONTEXT.md`.

## Language

**Van frame**:
The coordinate frame angles are reported in: +x forward, +y left, +z up. The board can be mounted in any orientation; `axis_x`/`axis_y`/`axis_z` in config map sensor axes onto it.
_Avoid_: Sensor frame, board frame (the raw MPU axes, which are never published).

**roll_deg**:
Rotation about the van's forward axis, degrees, positive = left side down. `atan2(ay, az)` of the van-frame gravity vector, smoothed, minus the level-zero offset.
_Avoid_: Bank, lean, tilt (the entity name; this is one of its two angles).

**pitch_deg**:
Rotation about the van's lateral axis, degrees, positive = nose up. `atan2(-ax, hypot(ay, az))` of the van-frame gravity vector, smoothed, minus the level-zero offset.
_Avoid_: Slope, incline, grade.

**Level zero**:
The install-time offset (one per angle) captured by publishing to `level_zero/set` while the van is parked level. Removes the residual error between the board's mounting and the van's true horizontal; persisted in `state/<id>.json`. `reset` clears it. Subtracted after smoothing, so capturing it never depends on the smoothing history.
_Avoid_: Calibration (it corrects mounting, not sensor scale/bias), tare.

**Dynamic sample**:
An accelerometer sample whose magnitude is outside `gravity_min_g`..`gravity_max_g` — braking, cornering, or a bump, where the sensor measures more than gravity so the angle is wrong. Skipped for smoothing once an estimate exists (the very first sample always seeds it).
_Avoid_: Bad sample, outlier.

**status**:
`ok` / `sensor_error`. Flips to `sensor_error` after `error_threshold` consecutive failed reads (I2C error or an all-zero accelerometer read); angles are not republished while in error. Published on transition only.
