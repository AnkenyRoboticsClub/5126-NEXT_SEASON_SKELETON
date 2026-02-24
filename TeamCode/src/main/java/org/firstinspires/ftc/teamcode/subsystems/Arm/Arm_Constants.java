package org.firstinspires.ftc.teamcode.subsystems.Arm;

public class Arm_Constants {


    // ---------------------------- Power / Speed ----------------------------
    public static final double ARM_SPEED              = 0.5;   // normal manual power multiplier
    public static final double ARM_SPEED_MAX          = 1.0;   // boosted speed (hold left_bumper)
    public static final double ARM_HOLD_POWER         = 0.001; // tiny power to hold position against gravity
    public static final double ARM_HOLD_POWER_OVER    = -0.001;// nudge down if somehow over max

    // ---------------------------- Encoder Limits ----------------------------
    public static final int ARM_MIN_POS               = 10;    // below this, treat as bottom — don't hold
    public static final int ARM_MAX_POS               = 939;   // soft upper limit

    // ---------------------------- Preset Positions ----------------------------
    // "B button" basket preset (shared with viper slide sequence)
    public static final int ARM_PRESET_BASKET         = 677;   // arm angle for basket deposit

    // Viper position threshold before arm is allowed to move during basket preset
    public static final int ARM_PRESET_VIPER_THRESHOLD = 900;  // wait for viper > this before moving arm
    public static final double ARM_PRESET_VELOCITY    = 800;   // ticks/sec for RUN_TO_POSITION

    // ---------------------------- Deadband ----------------------------
    public static final double ARM_STICK_DEADBAND     = 0.1;   // ignore stick input below this

}
