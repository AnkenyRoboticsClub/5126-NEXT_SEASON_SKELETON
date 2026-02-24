# FTC Next Season Skeleton (TeamCode Starter)

This repo is a **starting framework** for next season’s coders.
It includes reusable subsystem classes, constants, and one basic TeleOp so the next team can plug in hardware names and start driving quickly.

## What’s already in this skeleton

### Included subsystems
- **Mechanum Drive** (field-centric + robot-centric drive helpers, nudge/scootch helpers)
  - `TeamCode/.../subsystems/MechanumDrive/MechanumDrive_Subsystem.java`
  - `TeamCode/.../subsystems/MechanumDrive/MechanumDrive_Constants.java`
- **Viper Slide** (manual, hold power logic, presets)
  - `TeamCode/.../subsystems/ViperSlide/ViperSlide_Subsystem.java`
  - `TeamCode/.../subsystems/ViperSlide/ViperSlide_Constants.java`
- **Arm** (manual + hold logic)
  - `TeamCode/.../subsystems/Arm/Arm_Subsystem.java`
  - `TeamCode/.../subsystems/Arm/Arm_Constants.java`
- **Flywheel** (RPM-based velocity control)
  - `TeamCode/.../subsystems/FlyWheel/Flywheel_Subsystem.java`
  - `TeamCode/.../subsystems/FlyWheel/Flywheel_Constants.java`
- **Limelight** (tag aiming helpers)
  - `TeamCode/.../subsystems/LimeLight/LimeLight_Subsystem.java`
  - `TeamCode/.../subsystems/LimeLight/LimeLight_Constants.java`

### Included OpModes
- **`Basic TeleOp`**
  - Class: `TeamCode/.../teleop/BasicTeleOp.java`
  - Purpose: basic field-centric mecanum drive with speed modes and nudge controls.

---

## First-time setup checklist (IMPORTANT)

### 1) Configure hardware names
Open:
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/common/Robot_Constants.java`

Replace every `"REPLACE_ME"` with the exact names from the Robot Controller configuration.

#### Hardware name map in this project
| Constant | Expected hardware |
|---|---|
| `M_FL`, `M_FR`, `M_BL`, `M_BR` | Drive motors |
| `M_FLY` | Flywheel motor |
| `M_INTAKE` | Intake motor *(constant exists; subsystem template not added yet)* |
| `V_RIGHT`, `V_LEFT` | Viper slide motors |
| `ARM` | Arm motor |
| `S_KICK`, `S_INTAKE` | Servos *(constants exist; subsystem template not added yet)* |
| `PINPOINT` | Pinpoint localizer device name |
| `IMU` | IMU device name (default `"imu"`) |
| `LL_DEVICE_NAME` | Limelight device name (default `"limelight"`) |

> If names do not match exactly, your OpMode will crash at init when hardware is fetched.

### 2) Verify motor directions
In each subsystem constructor, confirm `.setDirection(...)` matches your real robot wiring/build. This is especially important for:
- `MechanumDrive_Subsystem`
- `ViperSlide_Subsystem`
- `Flywheel_Subsystem`

### 3) Tune constants before driving hard
Review constants files and tune for your robot:
- `MechanumDrive_Constants` (speed scales, scootch)
- `ViperSlide_Constants` (limits, presets, hold power)
- `Arm_Constants` (limits, hold power)
- `Flywheel_Constants` (RPM filtering/tolerance)
- `LimeLight_Constants` (aim/approach gains and tolerances)

### 4) Build + deploy from Android Studio
- Open this project in Android Studio.
- Sync Gradle.
- Build and run to Robot Controller phone/hub as normal FTC workflow.

---

## Current driver controls in `Basic TeleOp`

### Driver 1 (gamepad1)
- `left stick` = translation (field-centric)
- `right stick x` = turn
- `options` = reset IMU yaw
- `right trigger` = fast mode
- `left trigger` = slow mode
- `dpad up/down/left/right` = short “nudge/scootch” movement

Telemetry currently prints per-wheel RPM + average RPM.

---

## “Commands already in here” (subsystem APIs)

This skeleton is subsystem-first. The main “commands” are methods you call from OpModes.

### Drive (Mechanum)
- `driveFieldCentric(...)`
- `driveRobot(...)`
- `nudgeForward()/nudgeBack()/nudgeLeft()/nudgeRight()`
- `assistLeft()/assistRight()`
- `stopAll()`

### Viper Slide
- `update(gamepad2)` (manual + presets)
- `resetEncoders()`
- `getPosition()` and telemetry helpers

### Arm
- `update(gamepad2)`
- `resetEncoders()`
- `getPosition()` and telemetry helpers

### Flywheel
- `setFlywheelRpm(...)`
- `closeShoot()`, `farShoot()`
- `intakeFW()`, `stop()`
- `update()`, `flywheelAtSpeed()`, `waitForAtSpeed(...)`

### Limelight
- `start(hardwareMap)`, `stop()`
- `faceTagStepRobotCentric()`
- `aimAndApproachStepRobotCentric()`
- `faceTagUntil(...)`, `aimAndApproachUntil(...)`
- `scanMotif()`, `getTagId()`, `getDistance()`

---

## How to add a new subsystem

1. Create folder: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/<YourSubsystem>/`
2. Add `<YourSubsystem>_Subsystem.java` and `<YourSubsystem>_Constants.java`.
3. Add hardware name constant(s) in `Robot_Constants.java`.
4. Initialize subsystem in your OpMode:
   - constructor in `runOpMode()` init section
   - call `update(...)` methods inside loop
5. Add telemetry outputs so future coders can debug quickly.
6. Add a section in this README for controls/API.

---

## How to remove a subsystem cleanly

1. Remove subsystem files from `subsystems/<Name>/`.
2. Remove its constants from `Robot_Constants.java`.
3. Remove imports/usages in any OpMode.
4. Build project to confirm no references remain.
5. Update this README so docs match code.

---

## Subsystems you may still want to add (recommended)

Since next season’s game is unknown, these are common FTC templates worth adding:
- **Intake subsystem** (motor + servo state machine)
- **Outtake/Depositor subsystem** (claw/wrist linkage control)
- **Climber/Hang subsystem**
- **LED/status subsystem** (driver feedback)
- **Sensor wrapper subsystem** (distance/color/touch with filtered reads)
- **Command/state machine layer** (high-level actions like “pickup”, “score”, “stow”)

This repo already has constants for some intake-related hardware, so intake is a natural next subsystem to implement first.

---

## Suggested coding conventions for next-year coders

- Keep one subsystem per folder with:
  - `*_Subsystem.java`
  - `*_Constants.java`
- Keep hardware names centralized in `Robot_Constants.java`.
- Put driver control mapping in OpMode, not buried deep in subsystem internals.
- Make every subsystem expose:
  - `update(...)` (or clear command methods)
  - `addTelemetry(...)`
  - safe defaults (`stop`, hold logic, soft limits)

---

## Quick handoff note

If you are reading this next season:
1. Start by renaming hardware constants in `Robot_Constants.java`.
2. Test `Basic TeleOp` driving only.
3. Bring up one subsystem at a time (viper, arm, then others).
4. Tune constants slowly and commit often.

Good luck and build something awesome 🚀
