# LimeLight Vision Subsystem — Beginner's Guide

This guide explains what the LimeLight is, how it works, and how to use it in your code.

---

## What is the LimeLight?

The **LimeLight 3A** is a small AI camera that mounts on the robot. It can:
- Detect **AprilTags** (special square markers placed around the field)
- Tell you the **position and angle** of a tag relative to the robot
- Help the robot **automatically aim and drive** toward a target

Think of it as the robot's "eye" — it sees the field and your code decides what to do with that information.

---

## How Does It Work?

### AprilTags
AprilTags are like QR codes for robots. Each tag has a unique **ID number**. In our game:

| Tag ID | Meaning |
|--------|---------|
| 21     | GPP motif |
| 22     | PGP motif |
| 23     | PPG motif |

The LimeLight detects these tags and reports back a **3D pose** — essentially an (x, y, z) position and rotation describing where the tag is relative to the camera.

### Robot-Space Coordinates
When we say "robot space," we mean the coordinates are relative to **the robot itself**, not the field. So:
- `x` = how far **forward** the tag is
- `y` = how far **sideways** the tag is

From those two values, we can compute a **bearing angle** — how many degrees the robot needs to turn to face the tag directly.

---

## The `LimeLight_Subsystem` Class

This class wraps all the LimeLight logic so you don't have to deal with the low-level camera API.

### Setup (called once at the start of an OpMode)

```java
// In your OpMode's init():
limelightSubsystem.start(hardwareMap);

// In your OpMode's stop() or end:
limelightSubsystem.stop();
```

---

## Key Methods You'll Actually Use

### Scanning for Motifs

```java
String motif = limelightSubsystem.scanMotif();
// Returns "GPP", "PGP", "PPG", or "UNKNOWN"
```

This reads the visible AprilTag ID and maps it to the game motif. Use this during autonomous to decide your strategy.

### Facing the Goal Tag (TeleOp / Autonomous)

These methods make the robot **rotate** until it's pointing at the goal tag.

```java
// Call this in a loop — returns true when aligned
boolean aligned = limelightSubsystem.faceTagStepRobotCentric();

// Or, block until aligned (for LinearOpMode autonomous)
limelightSubsystem.faceTagUntil(this); // 'this' = your LinearOpMode
```

### Aim AND Drive Toward the Tag

```java
// Call in a loop — returns true when both aimed AND close enough
boolean done = limelightSubsystem.aimAndApproachStepRobotCentric();

// Or block until complete (autonomous)
limelightSubsystem.aimAndApproachUntil(this);
```

### Get Distance to the Tag

```java
double distanceInches = limelightSubsystem.getDistance();
// Returns -1 if no tag is visible
```

### Get the Current Tag ID

```java
int id = limelightSubsystem.getTagId();      // goal tags only
int id = limelightSubsystem.getAnyTagId();   // any visible tag
// Returns -1 if nothing is visible
```

---

## Example: Autonomous OpMode

```java
@Autonomous
public class MyAutoOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        // ... initialize subsystems ...

        waitForStart();

        // 1. Scan the motif from the signal tag
        String motif = limelightSubsystem.scanMotif();
        telemetry.addData("Motif", motif);
        telemetry.update();

        // 2. Face the goal tag
        limelightSubsystem.faceTagUntil(this);

        // 3. Aim and drive toward the goal
        limelightSubsystem.aimAndApproachUntil(this);

        // 4. Do your scoring action here
    }
}
```

---

## How the Auto-Aim Works (Under the Hood)

You don't *need* to understand this to use the subsystem, but it helps when debugging.

1. **Get latest camera result** — the LimeLight polls at 100 Hz
2. **Find the goal tag** — searches the list of visible tags for the goal ID
3. **Compute bearing** — uses `atan2(y, x)` on the tag's robot-space position to get the angle offset in degrees
4. **Smooth the bearing** — a low-pass filter prevents the robot from twitching due to noisy readings
5. **Compute turn command** — proportional control (`bearing × kTurn`), with a minimum power bump to overcome static friction, clamped to a max speed, and slew-rate limited so it can't change too fast
6. **Drive the robot** — calls `drive.driveRobot(0, 0, turn)` for turning-only, or `drive.driveRobot(0, fwd, turn)` when approaching

---

## Common Issues

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| `scanMotif()` returns `"UNKNOWN"` | Tag not visible, wrong ID, or camera not started | Check camera position, make sure `start()` was called |
| Robot spins in the wrong direction | `LL_TURN_DIRECTION` constant is backwards | Flip it in `LimeLight_Constants` |
| Robot oscillates / hunts back and forth | Gains too high or deadband too small | Tune `LL_K_TURN` and `BEARING_DEADBAND_DEG` in constants |
| `getDistance()` returns `-1` | No valid tag visible | Check tag ID, lighting, camera angle |
| Robot overshoots the tag | Slew rate or max turn too high | Lower `LL_MAX_TURN` or `TURN_SLEW_PER_LOOP` |

---

## Tuning Constants

All tunable values live in `LimeLight_Constants.java`. The most important ones:

| Constant | What It Does |
|----------|-------------|
| `LL_K_TURN` | How aggressively to turn toward the tag |
| `LL_MIN_TURN` | Minimum motor power to overcome static friction |
| `LL_MAX_TURN` | Maximum turn speed |
| `BEARING_DEADBAND_DEG` | Stop turning when within this many degrees |
| `LL_AIM_TOL_DEG` | "Good enough" tolerance to report alignment done |
| `BEARING_ALPHA` | Low-pass filter strength (0–1, lower = smoother but slower) |
| `TURN_SLEW_PER_LOOP` | Max turn change per loop iteration |

---

## Quick Reference

```
Start camera      → limelightSubsystem.start(hardwareMap)
Scan motif        → limelightSubsystem.scanMotif()
Face tag (step)   → limelightSubsystem.faceTagStepRobotCentric()
Face tag (block)  → limelightSubsystem.faceTagUntil(this)
Aim + approach    → limelightSubsystem.aimAndApproachUntil(this)
Get distance      → limelightSubsystem.getDistance()
Stop camera       → limelightSubsystem.stop()
```

Good luck, and don't hesitate to ask a veteran coder if something is unclear!