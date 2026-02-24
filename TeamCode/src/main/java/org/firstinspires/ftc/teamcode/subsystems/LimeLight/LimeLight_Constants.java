package org.firstinspires.ftc.teamcode.subsystems.LimeLight;

public class LimeLight_Constants {
    // ---------------------------- Tuning knobs ----------------------------

    // Your camera is RIGHT SIDE UP. Leave this false.
    // If you ever mount the Limelight 180° (upside down), set true.
    public static final boolean CAMERA_UPSIDE_DOWN = false;

    // Noise deadband (deg). Under this, we do 0 turn to avoid wiggle.
    public static final double BEARING_DEADBAND_DEG = 1.5;

    // "Close enough" to stop (deg). If you already have this in RobotConstants, keep using it.
    // We'll use RobotConstants.LL_AIM_TOL_DEG as the final "done" check.

    // Simple smoothing for bearing to kill jitter (0..1). Higher = more responsive, lower = smoother.
    public static final double BEARING_ALPHA = 0.35;

    // Optional: cap how fast the turn command can change each loop (prevents snap)
    public static final double TURN_SLEW_PER_LOOP = 0.08;
    // -------------------- LIMELIGHT CONSTANTS --------------------

    // Device name from RC config
    public static final String LL_DEVICE_NAME = "limelight";

    // Tuning multipliers
    // Turn direction for LL corrections. Use +1.0 or -1.0 depending on drivetrain/IMU convention.
    public static final double LL_TURN_DIRECTION = -1.0;

    public static final double LL_K_TURN      = 0.035;   // turning gain
    public static final double LL_K_FORWARD   = 0.25;    // forward gain

    // Minimum motor power so robot “breaks static friction”
    public static final double LL_MIN_TURN    = 0.17;
    public static final double LL_MIN_FORWARD = 0.20;

    // Maximum motor speeds for LL control
    public static final double LL_MAX_TURN    = 0.35;
    public static final double LL_MAX_FORWARD = 0.35;

    // TA value when robot is at perfect shooting distance
    // ★ YOU MUST UPDATE THIS using LLDebug ★
    public static double LL_TARGET_AREA = 1.45;

    // Goal AprilTag IDs
    public static final int LL_BLUE_GOAL_TAG_ID = 20;
    public static final int LL_RED_GOAL_TAG_ID  = 24;

    // Tolerances for stopping
    public static final double LL_AIM_TOL_DEG      = 1.5;
    public static final double LL_APPROACH_TOL_TA  = 0.05;

    // Auto alignment timeouts
    public static final double LL_ALIGN_TIMEOUT_S    = 2.0;
    public static final double LL_APPROACH_TIMEOUT_S = 3.0;

    double limelightLensHeightInches = 10.53;
    double goalHeightInches = 29.5;
}
