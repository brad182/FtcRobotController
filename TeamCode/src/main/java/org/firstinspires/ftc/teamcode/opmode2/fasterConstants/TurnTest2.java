package org.firstinspires.ftc.teamcode.opmode2.fasterConstants;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
public class TurnTest2 extends LinearOpMode {
    public static double ANGLE = 180; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive2 drive = new SampleMecanumDrive2(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.turn(Math.toRadians(ANGLE));
    }
}
