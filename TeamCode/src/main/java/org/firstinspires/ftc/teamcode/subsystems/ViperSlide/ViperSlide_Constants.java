package org.firstinspires.ftc.teamcode.subsystems.ViperSlide;

public class ViperSlide_Constants {

    // ---------------------------- Power / Speed ----------------------------
    public static final double VIPER_SPEED            = 1.0;   // default manual power multiplier
    public static final double VIPER_VELOCITY_NORMAL  = 1000;  // ticks/sec for RUN_TO_POSITION (basket)
    public static final double VIPER_VELOCITY_FAST    = 2000;  // ticks/sec for RUN_TO_POSITION (specimen)
    public static final double VIPER_HOLD_POWER       = 0.001; // small power to hold position against gravity
    public static final double VIPER_DOWN_POWER       = -1.0;  // forced down when over hard limit

    // ---------------------------- Encoder Limits ----------------------------
    public static final int VIPER_MIN_POS             = 0;     // fully retracted (encoder zero)
    public static final int VIPER_BOTTOM_GUARD        = 500;   // below this, hold power = 0 (don't fight the hard stop)
    public static final int VIPER_MAX_POS             = 4200;  // soft upper limit — above this, drive down

    // ---------------------------- Preset Positions ----------------------------
    // "B button" basket preset
    public static final int VIPER_PRESET_BASKET       = 980;   // viper height for basket deposit
    // "D-pad right" specimen preset
    public static final int VIPER_PRESET_SPECIMEN     = 3900;  // viper height for specimen hang

    // ---------------------------- Deadband ----------------------------
    public static final double VIPER_STICK_DEADBAND   = 0.1;   // ignore stick input below this
}
