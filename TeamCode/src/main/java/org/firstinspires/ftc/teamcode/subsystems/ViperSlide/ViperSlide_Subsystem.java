package org.firstinspires.ftc.teamcode.subsystems.ViperSlide;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.Robot_Constants;
import org.firstinspires.ftc.teamcode.subsystems.ViperSlide.ViperSlide_Constants;

/**
 * ViperSlide subsystem.
 *
 * Usage in your OpMode loop:
 *
 *   viperSlide.update(gamepad2);          // call once per loop for manual + preset control
 *   viperSlide.addTelemetry(telemetry);   // optional debug data
 */
public class ViperSlide_Subsystem {

    private final DcMotorEx right;
    private final DcMotorEx left;

    // -------------------------------------------------------------------------

    public ViperSlide_Subsystem(HardwareMap hw) {
        right  = (DcMotorEx) hw.dcMotor.get(Robot_Constants.V_RIGHT);
        left = (DcMotorEx) hw.dcMotor.get(Robot_Constants.V_LEFT);
        // Directions
        right.setDirection(DcMotorSimple.Direction.FORWARD);
        left.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset encoders on init
        resetEncoders();
    }

    // -------------------------------------------------------------------------
    //  Public API
    // -------------------------------------------------------------------------

    /** Call once at the start of your OpMode (after waitForStart if you prefer). */
    public void resetEncoders() {
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Main update — call once per loop iteration.
     * Handles manual stick, hold logic, soft limits, and presets.
     *
     * Button map (gamepad2):
     *   right_stick_y  → manual control
     *   b              → basket preset  (hold button)
     *   dpad_right     → specimen preset (hold button)
     *   back           → reset encoders
     */
    public void update(Gamepad gamepad2) {

        // -- Encoder reset --
        if (gamepad2.back) {
            resetEncoders();
            return;
        }

        // -- Basket preset (hold B) --
        if (gamepad2.b) {
            runToPosition(ViperSlide_Constants.VIPER_PRESET_BASKET,
                    ViperSlide_Constants.VIPER_VELOCITY_NORMAL);
            return;
        }

        // -- Specimen preset (hold dpad_right) --
        if (gamepad2.dpad_right) {
            runToPosition(ViperSlide_Constants.VIPER_PRESET_SPECIMEN,
                    ViperSlide_Constants.VIPER_VELOCITY_FAST);
            return;
        }

        // -- Manual control --
        double stickInput = -gamepad2.right_stick_y;

        if (Math.abs(stickInput) > ViperSlide_Constants.VIPER_STICK_DEADBAND) {
            // Driver is actively moving — let them
            setManualPower(stickInput * ViperSlide_Constants.VIPER_SPEED);
        } else {
            // Stick released — apply hold / limit logic
            applyHoldLogic();
        }
    }

    /** Convenience: current position of the right motor (use as reference). */
    public int getPosition() {
        return right.getCurrentPosition();
    }

    public int getLeftPosition() {
        return left.getCurrentPosition();
    }

    public int getPositionDiff() {
        return Math.abs(left.getCurrentPosition() - right.getCurrentPosition());
    }

    /** Add viper slide debug lines to telemetry. */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("ViperRight Pos", right.getCurrentPosition());
        telemetry.addData("ViperLeft  Pos", left.getCurrentPosition());
        telemetry.addData("Viper Diff",     getPositionDiff());
    }

    // -------------------------------------------------------------------------
    //  Private helpers
    // -------------------------------------------------------------------------

    private void setManualPower(double power) {
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setPower(power);
        left.setPower(power);
    }

    private void applyHoldLogic() {
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int pos = getPosition();
        double holdPower;

        if (pos > ViperSlide_Constants.VIPER_MAX_POS) {
            // Way too high — force it back down
            holdPower = ViperSlide_Constants.VIPER_DOWN_POWER;
        } else if (pos <= ViperSlide_Constants.VIPER_BOTTOM_GUARD) {
            // Near the bottom hard stop — don't fight it
            holdPower = 0;
        } else {
            // Normal range — apply a tiny hold power to fight gravity
            holdPower = ViperSlide_Constants.VIPER_HOLD_POWER;
        }

        right.setPower(holdPower);
        left.setPower(holdPower);
    }

    private void runToPosition(int targetTicks, double velocity) {
        // Read current position before switching modes
        right.getCurrentPosition();
        left.getCurrentPosition();

        right.setTargetPosition(targetTicks);
        left.setTargetPosition(targetTicks);

        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        right.setVelocity(velocity);
        left.setVelocity(velocity);
    }
}