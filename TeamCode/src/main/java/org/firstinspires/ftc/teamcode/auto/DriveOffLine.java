package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive.AutoDrive;

@Autonomous(name="Drive Off Line", group="Auto")
public class DriveOffLine extends LinearOpMode {

    @Override
    public void runOpMode() {
        AutoDrive drive = new AutoDrive(hardwareMap);

        telemetry.addLine("Auto Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        drive.driveStraightInches(this, 12, 0.5);
        sleep(2000);
        drive.stopAll();

        telemetry.addLine("Auto Done");
        telemetry.update();
        sleep(250);
    }
}
