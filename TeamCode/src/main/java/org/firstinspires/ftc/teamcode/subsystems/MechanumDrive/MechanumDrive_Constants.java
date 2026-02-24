package org.firstinspires.ftc.teamcode.subsystems.MechanumDrive;

public class MechanumDrive_Constants {
    private MechanumDrive_Constants() {}
    static final double TICKS_PER_REV = 537.6;
    // Drive speed scales, power # is from 0 -> 1, 0=0% and 1=100%
    public static final double TURN_SPEED          = 0.7; //Prob should stay the same as default speed, honestly idk
    public static final double MOVING_SPEED_SLOW   = 0.4;
    public static final double MOVING_SPEED        = 0.8; //Default Speed
    public static final double MOVING_SPEED_FAST   = 1.0; //Usualy max power

    // Nudge (“scootch”)
    public static final double SCOOTCH_POWER       = 0.5;   // tune for more powerfull scootch
    public static final long   SCOOTCH_DURATION_MS = 200;   // tune for longer or shorter scootch

    public static final double TICKS_PER_MOTOR_REV = 28.0 * 4.0; // = 112
    public static final double GEAR_RATIO = 49.0 / 10.0;  // 4.8 : 1
    public static final double WHEEL_DIAMETER_IN = 4.0944882; //or 104mm i think
    public static final double WHEEL_CIRCUMFERENCE_IN =
            Math.PI * WHEEL_DIAMETER_IN;
    public static final double TICKS_PER_INCH =
            (TICKS_PER_MOTOR_REV * GEAR_RATIO) / WHEEL_CIRCUMFERENCE_IN;
    public static int inchesToTicks(double inches) {
        return (int) Math.round(inches * TICKS_PER_INCH);
    }
}
