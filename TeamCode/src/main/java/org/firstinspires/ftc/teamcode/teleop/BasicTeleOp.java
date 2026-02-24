package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive.MecanumDrive_Subsystem;
import org.firstinspires.ftc.teamcode.common.ImuUtil;


@TeleOp(name="Basic TeleOp", group="Linear OpMode")
public class BasicTeleOp extends LinearOpMode {

    private MecanumDrive_Subsystem drive;
    private ImuUtil imu;

    @Override
    public void runOpMode() {
        drive   = new MecanumDrive_Subsystem(hardwareMap);
        imu     = new ImuUtil(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {

            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x;

            if (gamepad1.options) imu.resetYaw();
            double heading = imu.getHeadingRad();

            boolean fast = gamepad1.right_trigger > 0.1;
            boolean slow = gamepad1.left_trigger  > 0.1;

            // Driver 1 aim-assist is non-blocking while A is held
            drive.driveFieldCentric(x, y, rx, heading, slow, fast, false);

            if (gamepad1.dpad_right) drive.nudgeRight(this);
            if (gamepad1.dpad_left)  drive.nudgeLeft(this);
            if (gamepad1.dpad_up)    drive.nudgeForward(this);
            if (gamepad1.dpad_down)  drive.nudgeBack(this);
            //=============================

            telemetry.addLine("--- DRIVE RPM ---");
            telemetry.addData("FL", "%.1f", drive.getRPM(drive.fl));
            telemetry.addData("FR", "%.1f", drive.getRPM(drive.fr));
            telemetry.addData("BL", "%.1f", drive.getRPM(drive.bl));
            telemetry.addData("BR", "%.1f", drive.getRPM(drive.br));
            double avgAbs = (Math.abs(drive.getRPM(drive.fl)) + Math.abs(drive.getRPM(drive.fr)) +
                    Math.abs(drive.getRPM(drive.bl)) + Math.abs(drive.getRPM(drive.br))) / 4.0;
            telemetry.addData("Avg (abs)", "%.1f", avgAbs);
            telemetry.update();
        }
    }
}
