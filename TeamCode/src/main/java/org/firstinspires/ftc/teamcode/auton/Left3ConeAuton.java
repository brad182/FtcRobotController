package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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

@Autonomous(name = "Left3ConeAuton", group = "roadrunner") //@Autonomous(name = "Left3ConeAuton", group = "roadrunner", preselectTeleOp="TankArmClaw_2Drivers.java") USE NEXT SEASON
public class Left3ConeAuton extends LinearOpMode
{
    ElapsedTime time=new ElapsedTime();
    public int CurrentTargetAngle = 0;
    public DcMotor leftLiftMotor = null;
    public DcMotor rightLiftMotor = null;
    public Servo clawMotor = null;
    public Servo perpendicularEncoderLift = null;
    public Servo parallelEncoderLift = null;
    static int[] clawToggle = {0, 1};
    public Servo polePusher = null;
    public static final int HI = 2695; //hi value originally 2995
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
        double forwardAmount = 50;
        double forwardPole = 13.5;
        double forwardCone = 6;
        double turnAmount = -45;
        double waittime = 0.1;
        TrajectorySequence cycle = drive.trajectorySequenceBuilder(new Pose2d())  // drive forward to pole
                .strafeRight(1.5)
                .forward(53)
                .UNSTABLE_addTemporalMarkerOffset(-1.8, () -> lift(HI))
                .lineToLinearHeading(new Pose2d(forwardAmount+2, 0, 0))
                .back(2)
                .turn(Math.toRadians(turnAmount-1.25))  // more negative to turn right more
                  // Preload //// IMPORTANT: x and y are switched, y is negative (in relation to a coordinate plane)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> polePusher.setPosition(0.18))


                .forward(forwardPole-1.2)

                .waitSeconds(waittime)
                .UNSTABLE_addTemporalMarkerOffset(-0.8, () -> lift(HI-800)) //-100
                //.UNSTABLE_addTemporalMarkerOffset(-0.05, () -> lift(HI-500)) //trying to lower lift

                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> clawMotor.setPosition(0.5))

                .back(forwardPole)
                .UNSTABLE_addTemporalMarkerOffset(-.5, () -> polePusher.setPosition(0))
                //go to get stack for +1
                .lineToLinearHeading(new Pose2d(forwardAmount, 21, Math.toRadians(90)))
//lift doesnt go up on cycle 1, cycle 2 too far forward
//                .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> lift(00))
//                .UNSTABLE_addTemporalMarkerOffset(-1, () -> lift(150))
//                .forward(forwardCone)
//                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> clawMotor.setPosition(0))
//                .addDisplacementMarker(() -> {lift(600);})
//                .back(forwardPole)
//                //return to junction
//                .lineToLinearHeading(new Pose2d(forwardAmount, 0, Math.toRadians(turnAmount))) // first +1
//                .UNSTABLE_addTemporalMarkerOffset(-2, () -> lift(HI))
//                .forward(forwardPole)
//                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> clawMotor.setPosition(0.5))
//                .back(forwardPole)
//
//                //go to STACK +2
//                .lineToLinearHeading(new Pose2d(forwardAmount, -23, Math.toRadians(90)))

                .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> lift(00))
                .UNSTABLE_addTemporalMarkerOffset(-0.6, () -> lift(160))
                .forward(forwardCone)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> clawMotor.setPosition(0))
                .addDisplacementMarker(() -> {lift(850);})
                .back(forwardCone)

                //junction deposit +1
                .lineToLinearHeading(new Pose2d(forwardAmount, -1, Math.toRadians(90)))//Math.toRadians(turnAmount+1))) // first +1
                .turn(Math.toRadians(turnAmount-90))
                .UNSTABLE_addTemporalMarkerOffset(-1.8, () -> lift(HI))
                .forward(forwardPole-0.7)

                .UNSTABLE_addTemporalMarkerOffset(-.8, () -> polePusher.setPosition(0.17))
                .waitSeconds(waittime)

                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> lift(HI-100))

                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> clawMotor.setPosition(0.5))

                .back(forwardPole-0.7)
                .UNSTABLE_addTemporalMarkerOffset(-.5, () -> polePusher.setPosition(0))
                //go to STACK for +2
                .lineToLinearHeading(new Pose2d(forwardAmount+0.5, 23, Math.toRadians(90)))

                .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> lift(00))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> lift(130))
                .forward(forwardCone)

                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> clawMotor.setPosition(0))
                .addDisplacementMarker(() -> {lift(600);})
                .back(forwardCone)

                //junction deposit +2 on junction // NOTE: THIS IS DIFFERENT TO GET INTO POSITION FOR PARKING
                .lineToLinearHeading(new Pose2d(forwardAmount-0.25, -14, Math.toRadians(0)))//11.5
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> lift(HI))

                .forward(8)
                .UNSTABLE_addTemporalMarkerOffset(-.8, () -> polePusher.setPosition(0.17))
                .waitSeconds(waittime)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> lift(HI-100))

                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> clawMotor.setPosition(0.5))
                .back(9)
                .UNSTABLE_addTemporalMarkerOffset(-.5, () -> polePusher.setPosition(0))

                .build();
//VISION
        TrajectorySequence oneDotLeft = drive.trajectorySequenceBuilder(cycle.end())
                .lineToLinearHeading(new Pose2d(forwardAmount, 26, Math.toRadians(90)))//27
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> lift(00))
                .build();

        TrajectorySequence twoDotRight = drive.trajectorySequenceBuilder(cycle.end())
                .lineToLinearHeading(new Pose2d(forwardAmount, 2.5, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> lift(00))
                .build();

        TrajectorySequence threeDotRight = drive.trajectorySequenceBuilder(cycle.end())

                .lineToLinearHeading(new Pose2d(forwardAmount+2, -21.5, Math.toRadians(90)))//24
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> lift(00))
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
        polePusher = hardwareMap.get(Servo.class, "polePusher");
        perpendicularEncoderLift.setPosition(1); //These two were moved from the main run area to save time
        parallelEncoderLift.setPosition(1);
        clawMotor.setPosition(0);
        polePusher.setPosition(0);

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
            //drive.followTrajectory(visionShift);
            // cycling
            camera.closeCameraDevice();
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

            while  (true != drive.touchSensor.isPressed()){  // keep going down until sensor is pressed
                leftLiftMotor.setPower(-3);
                rightLiftMotor.setPower(-3);
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