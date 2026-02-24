package org.firstinspires.ftc.teamcode.subsystems.FlyWheel;

public class Flywheel_Constants {
    public static final double TICKS_PER_MOTOR_REV = 28.0 * 4.0; // = 112
    // Tuning
    public static final double RPM_ALPHA = 0.25; // 0..1 (higher = less smoothing)
    public static final double AT_SPEED_TOL_RPM = 50; // how close is "good enough"

    // Shooter tuning
    public static final double FLY_SPEED_SHOOT   = 1.0;  // forward shoot speed
    public static final double FLY_SPEED_REVERSE = -0.75; // gentle reverse for unjam

    public static final double FLY_CLOSE_RPM = 670; //A guess
    public static final double FLY_FAR_RPM = 1000; //A guess
}
