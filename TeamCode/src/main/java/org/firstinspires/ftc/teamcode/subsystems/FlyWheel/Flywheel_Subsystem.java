package org.firstinspires.ftc.teamcode.subsystems.FlyWheel;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.InterpolatingMap;
import org.firstinspires.ftc.teamcode.common.Robot_Constants;

public class Flywheel_Subsystem {


    private final DcMotorEx fly;      // <-- DcMotorEx so we can read velocity reliably

    private final InterpolatingMap FlywheelMap = new InterpolatingMap();
    PIDFCoefficients pid = new PIDFCoefficients(10, 3, 0, 12);

    // TeleOp helper state: fire only once while button is held
    private boolean distanceShotFiredThisHold = false;

    public double targetRpm = 0.0;
    public double rpmFiltered = 0.0;

    public Flywheel_Subsystem(HardwareMap hw) {
        fly  = (DcMotorEx) hw.dcMotor.get(Robot_Constants.M_FLY);

        fly.setDirection(DcMotorSimple.Direction.REVERSE);
        fly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Encoder needed for velocity
        fly.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fly.setVelocityPIDFCoefficients(10, 3, 0, 12);
        /*
         * FLYWHEEL VELOCITY PIDF TUNING NOTES
         *
         * If flywheel oscillates during initial spool-up:
         * 1. LOWER P first (most common cause).
         *    Try: P = 6 → 4 → 3
         *
         * 2. Add small D for damping if overshooting.
         *    Try: D = 0.5 → 1.0 → 2.0
         *
         * 3. Reduce or disable I while tuning.
         *    Large I (like 3) can cause wind-up during spool.
         *    Tune with I = 0 first, then add small value (0.2–0.5) only if needed.
         *
         * 4. Make sure F is close to correct.
         *    F should handle most of the steady-state velocity.
         *    If F is too low, PID has to fight harder → oscillation.
         *
         * Recommended test baseline:
         * fly.setVelocityPIDFCoefficients(4, 0, 1.0, 12);
         *
         * Tune order: F → P → D → small I (last)
         */

        fly.setPower(0);
        //         (Distance, power/rpm)
        FlywheelMap.put(0.0, 0.0);
    }

    private double getFlywheelRpmInstant() {
        double ticksPerSecond = fly.getVelocity();
        return (ticksPerSecond / Flywheel_Constants.TICKS_PER_MOTOR_REV) * 60.0;
    }

    /** Call this every loop (TeleOp/Auto) to keep rpmFiltered updated. */
    public void update() {
        double rpmNow = getFlywheelRpmInstant();
        rpmFiltered = (Flywheel_Constants.RPM_ALPHA * rpmNow) + ((1.0 - Flywheel_Constants.RPM_ALPHA) * rpmFiltered);
    }

    /** Measured (filtered) flywheel RPM. */
    public double getFlywheelRpm() {
        return rpmFiltered;
    }

    /** Last commanded target RPM (what you WANT). */
    public double getTargetRpm() {
        return targetRpm;
    }

    /** True when we're close enough to the target to confidently shoot. */
    public boolean atSpeed() {
        return Math.abs(getFlywheelRpmInstant() - targetRpm) <= Flywheel_Constants.AT_SPEED_TOL_RPM;
    }

    // ----- Flywheel controls -----
    public boolean flywheelAtSpeed(){
        return atSpeed();
    }


    /** Velocity control in encoder ticks/sec based on requested RPM. */
    public void setFlywheelRpm(double rpm) {
        targetRpm = Math.max(0, rpm);
        double ticksPerSecond = (targetRpm / 60.0) * Flywheel_Constants.TICKS_PER_MOTOR_REV;

        // Requires RUN_USING_ENCODER
        fly.setVelocity(ticksPerSecond);
    }



    /** If you still want raw power control sometimes. */
    public void setFlywheelPower(double p) {
        if (p > 1) p = 1;
        if (p < -1) p = -1;
        targetRpm = 0; // "no rpm target" when using power
        fly.setPower(p);
    }

    public void stop() {
        targetRpm = 0;
        fly.setPower(0);
    }

    public void intakeFW() {
        // reverse spinning; RPM target doesn’t really apply here
        targetRpm = 0;
        fly.setPower(Flywheel_Constants.FLY_SPEED_REVERSE);
    }

    // Presets (edit these to match your robot)
    public void closeShoot() { setFlywheelRpm(670); }
    public void farShoot()   { setFlywheelRpm(1000); }

    public void customShoot(double rpm, LinearOpMode op){
        setFlywheelRpm(rpm);
    }

    public boolean waitForAtSpeed(LinearOpMode op, long timeoutMs) {
        long start = System.currentTimeMillis();
        while (op.opModeIsActive() && (System.currentTimeMillis() - start) < timeoutMs) {
            update();
            if (flywheelAtSpeed()) {
                return true;
            }
            op.idle();
        }
        return false;
    }

    public double shootByDistance(double distance, LinearOpMode op){
        customShoot(FlywheelMap.get(distance), op);
        return FlywheelMap.get(distance);
    }
}
