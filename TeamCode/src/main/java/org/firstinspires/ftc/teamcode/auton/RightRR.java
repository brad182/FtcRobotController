package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "RightRR", group = "roadrunner")
/*
This file is essentially a copy paste from LeftRR, the only difference is that this one for #3 strafes left instead of right and the turns are negated from LeftRR
 */
public class RightRR extends LinearOpMode
{
    public int CurrentTargetAngle = 0;
    public DcMotor leftLiftMotor = null;
    public DcMotor rightLiftMotor = null;
    public Servo clawMotor = null;

    static int[] clawToggle = {0, 1};

    public static final int HI = 3000; //hi value
    public static final int GR = 00; //ground value

    int clawSwitch = 1;
    int liftPosition = 0;
    String v = "";

    OpenCvCamera camera;
    org.firstinspires.ftc.teamcode.auton.AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double speed = 0.9;
    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1, 2, 3 from the 36h11 family
    AprilTagDetection tagOfInterest = null;
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;




    @Override
    public void runOpMode()
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory forward1 = drive.trajectoryBuilder(new Pose2d())  // drive forward to pole
                .forward(52)
                .build();

        Trajectory backward2 = drive.trajectoryBuilder(new Pose2d()) // back up to get away from vision cone
                .back(2)
                .build();

        Trajectory strafeLeft3 = drive.trajectoryBuilder(new Pose2d())  // align with pole
                .strafeLeft(12)
                .build();

        Trajectory forward4 = drive.trajectoryBuilder(new Pose2d()) // forward to reach the pole
                .forward(2)
                .build();

        Trajectory backward5 = drive.trajectoryBuilder(new Pose2d()) // back up
                .back(4)
                .build();

        Trajectory forward6 = drive.trajectoryBuilder(new Pose2d()) // to cone stack
                .forward(30)
                .build();

        Trajectory backward7 = drive.trajectoryBuilder(new Pose2d()) // back to pole
                .back(30)
                .build();

        Trajectory forward8 = drive.trajectoryBuilder(new Pose2d()) // to pole
                .forward(3)
                .build();

        Trajectory oneDotLeft = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(36)
                .build();

        Trajectory twoDotLeft = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(12)
                .build();

        Trajectory threeDotRight = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(12)
                .build();

        leftLiftMotor = hardwareMap.get(DcMotor.class, "leftLiftMotor");
        rightLiftMotor = hardwareMap.get(DcMotor.class, "rightLiftMotor");
        clawMotor = hardwareMap.get(Servo.class, "clawMotor");
        leftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//    clawMotor.setPosition(1);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        //org.firstinspires.ftc.teamcode.auton.

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if (tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        if(opModeIsActive()) {
            leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            leftLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            // preload
            clawMotor.setPosition(0);
            sleep(1000);
            lift(450);

            drive.followTrajectory(forward1);
            drive.followTrajectory(backward2);
            drive.followTrajectory(strafeLeft3);
            lift(HI);
            drive.followTrajectory(forward4);
            sleep(500);
            clawMotor.setPosition(1);  // drop preload
            drive.followTrajectory(backward5);

            // start of cycle -------------------------------------------
            lift(GR);
            lift(260);
            sleep(100);
            drive.turn(Math.toRadians(-90));  // rotate clockwise
            drive.followTrajectory(forward6);  // reach cone stack

            clawMotor.setPosition(0);  // close claw
            sleep(300);
            lift(800);
            sleep(300);
            drive.followTrajectory(backward7);

            drive.turn(Math.toRadians(90));  // rotate counterclockwise to align with pole
            lift(HI);
            drive.followTrajectory(forward8);
            sleep(400);
            clawMotor.setPosition(1);  // drop cone
            drive.followTrajectory(backward5);
            // end of cycle ----------------------------------------------

            // start of 1 + 2 auton, save for after 1 + 1 is tested and if there is enough time

            /*
            // start of cycle -------------------------------------------
            lift(GR);
            lift(260);
            sleep(100);
            drive.turn(Math.toRadians(90));
            drive.followTrajectory(forward6);  // reach cone stack

            clawMotor.setPosition(0);  // close claw
            sleep(300);
            lift(800);
            sleep(300);
            drive.followTrajectory(backward7);

            drive.turn(Math.toRadians(-90));  // rotate clockwise to align with pole
            lift(HI);
            drive.followTrajectory(forward8);
            sleep(400);
            clawMotor.setPosition(1);  // drop cone
            drive.followTrajectory(backward5);
            // end of cycle ----------------------------------------------
            */

            //parking
            if (tagOfInterest == null || tagOfInterest.id == LEFT) {
                // pathing for one dot
                telemetry.addLine("One Dot");
                telemetry.update();

                drive.followTrajectory(oneDotLeft);
            }
            else if (tagOfInterest.id == MIDDLE) {
                // pathing for two dots
                telemetry.addLine("Two Dots");
                telemetry.update();

                drive.followTrajectory(twoDotLeft);
            }
            else {
                // pathing for three dots
                telemetry.addLine("Three Dots");
                telemetry.update();

                drive.followTrajectory(threeDotRight);
            }
        }
    }

    void tagToTelemetry (AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection                               .pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    private void lift (int height) {  // high
        leftLiftMotor.setTargetPosition(height);
        rightLiftMotor.setTargetPosition(height);
        leftLiftMotor.setPower(1);
        rightLiftMotor.setPower(1);
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}