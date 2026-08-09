# Gravity-fed rear shooter

This robot has no transfer, feeder, or gate subsystem. Fuel travels through the fixed intake and
gravity ramp directly into the fixed-angle, rear-facing shooter.

## Driver controls

- Left stick: field-relative translation
- Right stick X: chassis rotation
- Circle (hold): run CAN 10 forward at +3 V
- Triangle (hold): stop translating, aim the rear of the robot at the alliance Hub, then run the
  three shooter motors
- Square (hold): start the shooter sequence immediately without aiming or taking control of Swerve
- Create: reset field heading for the current alliance

CAN 9 runs forward at +7 V continuously whenever Teleop is enabled. It stops while disabled or in a
latched safety stop. The shooter remains off until heading error is at most 3 degrees and chassis
angular speed is at most 15 degrees per second for 0.1 seconds. If error grows beyond 6 degrees
while shooting, the three shooter outputs stop and the robot returns to aiming.

For both Triangle and Square shooting, shoot1 and shoot2 start first. After they have run
continuously for 1.0 second, shootup starts at +7 V. Stopping and starting a new shot resets this
one-second delay.

## Motor outputs

All listed mechanism motors use Phoenix 6 `VoltageOut`; there are no shooter followers, velocity
setpoints, or duty-cycle requests.

| Motor | CAN ID | Active output |
| --- | ---: | ---: |
| shoot1 | 30 | -7 V |
| shoot2 | 31 | +7 V |
| shootup | 32 | +7 V after shoot1/shoot2 have run for 1 s |
| iL always-on | 9 | +7 V in Teleop |
| IL Circle | 10 | +3 V while Circle is held |

## Before running on a real robot

Before loading fuel:

1. Raise the mechanism and verify that -7 V on shoot1 and +7 V on shoot2/shootup produce the intended
   physical directions.
2. Measure the Limelight robot-space transform and replace the provisional transform.
3. Confirm CAN 9 starts at Teleop enable and stops at disable.
4. Confirm Circle operates only CAN 10 in addition to the Teleop CAN 9 output.
5. Field-check every hard-coded autonomous point at low speed before running the full routine.

## Autonomous

Choose `Do Nothing`, `Shoot Only`, `Left Collect + Return`, or `Right Collect + Return` on the
Dashboard. `Shoot Only` resets to the alliance-transformed Blue-Left start, aims at the Hub, shoots
for the configured duration, and stops without collecting. The collect route is stored once as
Blue-Left coordinates; code mirrors it for the right side and rotates it for the red alliance.
`Auto/ShootSeconds` defaults to 3 seconds.

Every drive leg and aiming phase has a timeout. A timeout records an Auto fault and stops the drive,
intake, and shooter; later route legs are skipped.

## Elastic field display

The fused CTRE estimator pose is published as a WPILib `Field2d` at `/SmartDashboard/Field`.
Connect Elastic to the robot (or `localhost` in simulation), click **Add Widget**, expand
**SmartDashboard**, and drag **Field** onto the layout. Elastic should automatically select its
Field widget and show the robot translation and heading.
