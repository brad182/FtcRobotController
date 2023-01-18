package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Eric's Miraculous 2 Cone Attempt", group = "roadrunner")
public class EricsMiraculous2ConeAttempt extends LinearOpMode
{
    ElapsedTime time=new ElapsedTime();
    public int CurrentTargetAngle = 0;
    public DcMotor leftLiftMotor = null;
    public DcMotor rightLiftMotor = null;
    public Servo clawMotor = null;
    public Servo perpendicularEncoderLift = null;
    public Servo parallelEncoderLift = null;
    static int[] clawToggle = {0, 1};

    public static final int HI = 2995; //hi value
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

        //    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(-90)); OH THATS WHY IT WASN"T WORKING I WASN'T USING startPose!!!! AHA but I don't need it anymore so I'm commenting it out to avoid confusion
        Trajectory visionShift = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(2)
                .build();
        TrajectorySequence cycle = drive.trajectorySequenceBuilder(new Pose2d())  // drive forward to pole
                .lineToLinearHeading(new Pose2d(49, 0, Math.toRadians(45))) // Preload //// IMPORTANT: x and y are switched, y is negative (in relation to a coordinate plane)
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> clawMotor.setPosition(0)) // This may or may not need to be moved out into the beginning area
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> lift(HI))
                .forward(6)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> clawMotor.setPosition(1))
                .back(6)
                .lineToLinearHeading(new Pose2d(49, -21, Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> lift(235))
                .forward(4)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> clawMotor.setPosition(0))
                .back(4)
                .lineToLinearHeading(new Pose2d(49, 0, Math.toRadians(45))) // first +1
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> lift(HI))
                .forward(6)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> clawMotor.setPosition(1))
                .back(6)
                .lineToLinearHeading(new Pose2d(49, -21, Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> lift(235))
                .forward(4)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> clawMotor.setPosition(0))
                .back(4)
                .lineToLinearHeading(new Pose2d(49, 0, Math.toRadians(45))) // second +2
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> lift(HI))
                .forward(6)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> clawMotor.setPosition(1))
                .back(6)
                .lineToLinearHeading(new Pose2d(49, -21, Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> lift(235))
                .forward(4)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> clawMotor.setPosition(0))
                .back(4)
                .lineToLinearHeading(new Pose2d(49, 0, Math.toRadians(45))) // third +3
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> lift(HI))
                .forward(6)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> clawMotor.setPosition(1))
                .back(6)
                .lineToLinearHeading(new Pose2d(49, -21, Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> lift(235))
                .forward(4)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> clawMotor.setPosition(0))
                .back(4)
                .lineToLinearHeading(new Pose2d(49, 10.5, Math.toRadians(0))) // fourth +4 // NOTE: THIS IS DIFFERENT TO GET INTO POSITION FOR PARKING
                .forward(4)// probably needs to be 5?
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> clawMotor.setPosition(1))
                .back(4) // maybe needs to be 6?
                .build();

        TrajectorySequence oneDotLeft = drive.trajectorySequenceBuilder(cycle.end())
                .lineToLinearHeading(new Pose2d(49, 22, Math.toRadians(-180 + 1e-6)))
                .build();

        TrajectorySequence twoDotRight = drive.trajectorySequenceBuilder(cycle.end())
                .lineToLinearHeading(new Pose2d(49, 0, Math.toRadians(-180 + 1e-6)))
                .build();

        TrajectorySequence threeDotRight = drive.trajectorySequenceBuilder(cycle.end())
                .lineToLinearHeading(new Pose2d(49, -23, Math.toRadians(-180 + 1e-6)))
                .build();
        leftLiftMotor = hardwareMap.get(DcMotor.class, "leftLiftMotor");
        rightLiftMotor = hardwareMap.get(DcMotor.class, "rightLiftMotor");
        clawMotor = hardwareMap.get(Servo.class, "clawMotor");
        perpendicularEncoderLift = hardwareMap.get(Servo.class, "perpendicularEncoderLift");
        parallelEncoderLift = hardwareMap.get(Servo.class, "parallelEncoderLift");
        leftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        perpendicularEncoderLift.setPosition(1); //These two were moved from the main run area to save time
        parallelEncoderLift.setPosition(1);

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
            waitFor(20);
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
            //THIS IS THE MAIN RUN PLACE Heh it look so empty now
            //shift over
            drive.followTrajectory(visionShift);
            // cycling
            drive.followTrajectorySequence(cycle);


            //parking
            if (tagOfInterest == null || tagOfInterest.id == LEFT) {
                // pathing for one dot
                telemetry.addLine("One Dot");
                telemetry.update();

                drive.followTrajectorySequence(oneDotLeft);
            }
            else if (tagOfInterest.id == MIDDLE) {
                // pathing for two dots
                telemetry.addLine("Two Dots");
                telemetry.update();

                drive.followTrajectorySequence(twoDotRight);
            }
            else {
                // pathing for three dots
                telemetry.addLine("Three Dots");
                telemetry.update();

                drive.followTrajectorySequence(threeDotRight);
            }
            perpendicularEncoderLift.setPosition(0);
            parallelEncoderLift.setPosition(0);
            lift(00);
            lift(1);
            waitFor(2000);
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
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLiftMotor.setTargetPosition(height);
        rightLiftMotor.setTargetPosition(height);
        leftLiftMotor.setPower(1);
        rightLiftMotor.setPower(1);
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void waitFor (int x) {
        time.reset();
        while(time.milliseconds()<x)
        {}
    }
}

//package org.firstinspires.ftc.teamcode.auton;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//import org.openftc.apriltag.AprilTagDetection;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
//import java.util.ArrayList;
//
//@Autonomous(name = "Eric's Fail Tests", group = "roadrunner")
//public class TwoConeAttemptFails extends LinearOpMode
//{
//    ElapsedTime time=new ElapsedTime();
//    public int CurrentTargetAngle = 0;
//    public DcMotor leftLiftMotor = null;
//    public DcMotor rightLiftMotor = null;
//    public Servo clawMotor = null;
//    public Servo perpendicularEncoderLift = null;
//    public Servo parallelEncoderLift = null;
//    static int[] clawToggle = {0, 1};
//
//    public static final int HI = 2995; //hi value
//    public static final int GR = 00; //ground value
//
//    int clawSwitch = 1;
//    int liftPosition = 0;
//    String v = "";
//
//    OpenCvCamera camera;
//    org.firstinspires.ftc.teamcode.auton.AprilTagDetectionPipeline aprilTagDetectionPipeline;
//
//    static final double FEET_PER_METER = 3.28084;
//
//    // Lens intrinsics
//    // UNITS ARE PIXELS
//    // NOTE: this calibration is for the C920 webcam at 800x448.
//    // You will need to do your own calibration for other configurations!
//    double fx = 578.272;
//    double fy = 578.272;
//    double cx = 402.145;
//    double cy = 221.506;
//    double speed = 0.9;
//    // UNITS ARE METERS
//    double tagsize = 0.166;
//
//    // Tag ID 1, 2, 3 from the 36h11 family
//    AprilTagDetection tagOfInterest = null;
//    int LEFT = 1;
//    int MIDDLE = 2;
//    int RIGHT = 3;
//
//
//
//
//    @Override
//    public void runOpMode()
//    {
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(-90));
//
//        TrajectorySequence forward1 = drive.trajectorySequenceBuilder(new Pose2d())  // drive forward to pole
//                .lineToLinearHeading(new Pose2d(52, 0, Math.toRadians(45)))
//.UNSTABLE_addTemporalMarkerOffset(-2, () -> lift(HI))
//                .forward(6)
//                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> clawMotor.setPosition(1))
///*
//.addDisplacementMarker(() -> {
//    //clawMotor.setPosition(1);
//}
//*/
//                .back(7)
//                .lineToLinearHeading(new Pose2d(52, -21, Math.toRadians(-90)))
//                .forward(4)
//                .back(4)
////.UNSTABLE_addTemporalMarkerOffset(-2, () -> lift(235))
///*
//.addDisplacementMarker(() -> {
//    //clawMotor.setPosition(0);
//}
//*/
//
//
//// MOVE TO PARKING?
//                .back(6)
//                .lineToLinearHeading(new Pose2d(52, 0, Math.toRadians(45)))
//
//                .build();
//
//
//        Trajectory backward2 = drive.trajectoryBuilder(forward1.end()) // back up to get away from vision cone
//                .back(2.25)
//                .build();
//
//        Trajectory strafeLeft3 = drive.trajectoryBuilder(backward2.end())  // align with pole
//                .strafeLeft(11.3)
//                .build();
//
//        Trajectory forward4 = drive.trajectoryBuilder(strafeLeft3.end()) // forward to reach the pole
//                .forward(5.5)
//                .build();
//
//        Trajectory backward5 = drive.trajectoryBuilder(forward4.end()) // back up
//                .back(5)
//                .build();
//
//        Trajectory forward6 = drive.trajectoryBuilder(backward5.end().plus(new Pose2d(0, 0, Math.toRadians(-90))), false) // to cone stack
//                .forward(40.6)
//                .build();
//
//        Trajectory backward7 = drive.trajectoryBuilder(forward6.end()) // back to pole
//                .back(41.3)
//                .build();
//
//        Trajectory forward8 = drive.trajectoryBuilder(backward7.end().plus(new Pose2d(0, 0, Math.toRadians(90))), false) // to pole
//                .forward(5.5)
//                .build();
//        Trajectory backward9 = drive.trajectoryBuilder(forward8.end().plus(new Pose2d(0, 0, Math.toRadians(0))), false) // to pole
//                .back(6)
//                .build();
//
//
//        Trajectory oneDotLeft = drive.trajectoryBuilder(backward9.end().plus(new Pose2d(0, 0, Math.toRadians(0))), false)
//                .strafeLeft(12)
//                .build();
//
//        Trajectory twoDotRight = drive.trajectoryBuilder(backward9.end().plus(new Pose2d(0, 0, Math.toRadians(0))), false)
//                .strafeRight(12)
//                .build();
//
//        Trajectory threeDotRight = drive.trajectoryBuilder(backward9.end().plus(new Pose2d(0, 0, Math.toRadians(0))), false)
//                .strafeRight(34)
//                .build();
//        leftLiftMotor = hardwareMap.get(DcMotor.class, "leftLiftMotor");
//        rightLiftMotor = hardwareMap.get(DcMotor.class, "rightLiftMotor");
//        clawMotor = hardwareMap.get(Servo.class, "clawMotor");
//        perpendicularEncoderLift = hardwareMap.get(Servo.class, "perpendicularEncoderLift");
//        parallelEncoderLift = hardwareMap.get(Servo.class, "parallelEncoderLift");
//        leftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
////    clawMotor.setPosition(1);
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
//        //org.firstinspires.ftc.teamcode.auton.
//
//        camera.setPipeline(aprilTagDetectionPipeline);
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode)
//            {
//
//            }
//        });
//
//        telemetry.setMsTransmissionInterval(50);
//
//        /*
//         * The INIT-loop:
//         * This REPLACES waitForStart!
//         */
//        while (!isStarted() && !isStopRequested())
//        {
//            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
//
//            if(currentDetections.size() != 0)
//            {
//                boolean tagFound = false;
//
//                for(AprilTagDetection tag : currentDetections)
//                {
//                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
//                    {
//                        tagOfInterest = tag;
//                        tagFound = true;
//                        break;
//                    }
//                }
//                if(tagFound)
//                {
//                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
//                    tagToTelemetry(tagOfInterest);
//                }
//                else
//                {
//                    telemetry.addLine("Don't see tag of interest :(");
//
//                    if(tagOfInterest == null)
//                    {
//                        telemetry.addLine("(The tag has never been seen)");
//                    }
//                    else
//                    {
//                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                        tagToTelemetry(tagOfInterest);
//                    }
//                }
//
//            }
//            else
//            {
//                telemetry.addLine("Don't see tag of interest :(");
//
//                if(tagOfInterest == null)
//                {
//                    telemetry.addLine("(The tag has never been seen)");
//                }
//                else
//                {
//                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                    tagToTelemetry(tagOfInterest);
//                }
//
//            }
//
//            telemetry.update();
//            waitFor(20);
//        }
//
//        /*
//         * The START command just came in: now work off the latest snapshot acquired
//         * during the init loop.
//         */
//
//        /* Update the telemetry */
//        if (tagOfInterest != null)
//        {
//            telemetry.addLine("Tag snapshot:\n");
//            tagToTelemetry(tagOfInterest);
//            telemetry.update();
//        }
//        else
//        {
//            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
//            telemetry.update();
//        }
//
//        if(opModeIsActive()) {
//            leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            rightLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//            leftLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//
//            // preload
//
//            perpendicularEncoderLift.setPosition(1);
//            parallelEncoderLift.setPosition(1);
//
//
//            clawMotor.setPosition(0);
//            //perpendicularEncoderLift.setPosition(0);
//            //sleep(1250);
//            //lift(450);
//            waitFor(1000);
//            drive.followTrajectorySequence(forward1);
//
//            // end of cycle ----------------------------------------------
//
//            // start of 1 + 2 auton, save for after 1 + 1 is tested and if there is enough time
//
//            /*
//            // start of cycle -------------------------------------------
//            lift(GR);
//            lift(260);
//            waitFor(100);
//            drive.turn(Math.toRadians(90));
//            drive.followTrajectory(forward6);  // reach cone stack
//
//            clawMotor.setPosition(0);  // close claw
//            waitFor(300);
//            lift(800);
//            waitFor(300);
//            drive.followTrajectory(backward7);
//
//            drive.turn(Math.toRadians(-90));  // rotate clockwise to align with pole
//            lift(HI);
//            drive.followTrajectory(forward8);
//            waitFor(400);
//            clawMotor.setPosition(1);  // drop cone
//            drive.followTrajectory(backward5);
//            // end of cycle ----------------------------------------------
//            */
//
//            //parking
//            if (tagOfInterest == null || tagOfInterest.id == LEFT) {
//                // pathing for one dot
//                telemetry.addLine("One Dot");
//                telemetry.update();
//
//                drive.followTrajectory(oneDotLeft);
//            }
//            else if (tagOfInterest.id == MIDDLE) {
//                // pathing for two dots
//                telemetry.addLine("Two Dots");
//                telemetry.update();
//
//                drive.followTrajectory(twoDotRight);
//            }
//            else {
//                // pathing for three dots
//                telemetry.addLine("Three Dots");
//                telemetry.update();
//
//                drive.followTrajectory(threeDotRight);
//            }
//            perpendicularEncoderLift.setPosition(0);
//            parallelEncoderLift.setPosition(0);
//            lift(00);
//            lift(1);
//            waitFor(2000);
//        }
//
//    }
//
//    void tagToTelemetry (AprilTagDetection detection) {
//        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
//        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Y: %.2f feet", detection                               .pose.y*FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
//        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
//    }
//
//    private void lift (int height) {  // high
//        leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftLiftMotor.setTargetPosition(height);
//        rightLiftMotor.setTargetPosition(height);
//        leftLiftMotor.setPower(1);
//        rightLiftMotor.setPower(1);
//        leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }
//
//    private void waitFor (int x) {
//        time.reset();
//        while(time.milliseconds()<x)
//        {}
//    }
//}