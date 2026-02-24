package org.firstinspires.ftc.teamcode.subsystems.Arm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.Robot_Constants;
import org.firstinspires.ftc.teamcode.subsystems.Arm.Arm_Constants;

/**
 * Arm subsystem.
 *
 * Usage in your OpMode loop:
 *
 *   arm.update(gamepad2);                     // manual control + hold logic
 *   arm.updateWithViperPreset(gamepad2, viperSlide.getPosition()); // if using basket preset
 *   arm.addTelemetry(telemetry);              // optional debug data
 */
public class Arm_Subsystem {

    private final DcMotorEx motor;

    // -------------------------------------------------------------------------

    public Arm_Subsystem(HardwareMap hw) {
        motor  = (DcMotorEx) hw.dcMotor.get(Robot_Constants.ARM);
        resetEncoders();
    }

    // -------------------------------------------------------------------------
    //  Public API
    // -------------------------------------------------------------------------

    /**
     * Reset encoder — call during init.
     */
    public void resetEncoders() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Standard update — call once per loop.
     * <p>
     * Button map (gamepad2):
     * left_stick_y   → manual control
     * left_bumper    → hold for full speed manual
     * <p>
     * Note: if you use the basket preset (B button) which coordinates the arm
     * with the viper slide, use updateWithViperPreset() instead.
     */
    public void update(Gamepad gamepad2) {
        double speed = gamepad2.left_bumper ? Arm_Constants.ARM_SPEED_MAX : Arm_Constants.ARM_SPEED;
        double stickInput = -gamepad2.left_stick_y;

        if (Math.abs(stickInput) > Arm_Constants.ARM_STICK_DEADBAND) {
            setManualPower(stickInput * speed);
        } else {
            applyHoldLogic();
        }
    }

    public int getPosition() {
        return motor.getCurrentPosition();
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Arm Position", motor.getCurrentPosition());
    }

    // -------------------------------------------------------------------------
    //  Private helpers
    // -------------------------------------------------------------------------

    private void setManualPower(double power) {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(power);
    }

    private void applyHoldLogic() {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int pos = motor.getCurrentPosition();

        if (pos > Arm_Constants.ARM_MAX_POS) {
            motor.setPower(Arm_Constants.ARM_HOLD_POWER_OVER);
        } else if (pos <= Arm_Constants.ARM_MIN_POS) {
            // At the bottom hard stop — don't fight it
            motor.setPower(0);
        } else {
            motor.setPower(Arm_Constants.ARM_HOLD_POWER);
        }
    }
}
