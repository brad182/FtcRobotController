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

@Autonomous(name = "AlternateRight3Cone", group = "roadrunner")
public class AlternateRight3Cone extends LinearOpMode
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
        double forwardAmount = 50;
        double forwardPole = 13.8;
        double forwardCone = 5.4;
        double turnAmount = 46.2;
        double waittime = 0.2;

        // use
        // .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> lift(00))
        // to perform an action x amount of sections before the last call
        TrajectorySequence cycle = drive.trajectorySequenceBuilder(new Pose2d())  // drive forward to pole
                .forward(50)
                .turn(90)  // turn left 90
                .forward(24) // reach high junction
                .turn(45) // FIX TURN VALUE SIGN, SHOULD ROTATE 45 to reach the junction
                // Preload //// IMPORTANT: x and y are switched, y is negative (in relation to a coordinate plane)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> polePusher.setPosition(0.15))

                .UNSTABLE_addTemporalMarkerOffset(-1.8, () -> lift(HI))
                .forward(forwardPole-3.3) // reach junction

                .waitSeconds(waittime)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> clawMotor.setPosition(0.5))

                .back(forwardPole-3.3)
                .UNSTABLE_addTemporalMarkerOffset(-.5, () -> polePusher.setPosition(0))
                //go to get stack for +1
                .lineToLinearHeading(new Pose2d(forwardAmount, -23, Math.toRadians(-90)))


                .build();
//VISION
        TrajectorySequence oneDotLeft = drive.trajectorySequenceBuilder(cycle.end())
                .lineToLinearHeading(new Pose2d(forwardAmount, 24, Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> lift(00))
                .build();

        TrajectorySequence twoDotRight = drive.trajectorySequenceBuilder(cycle.end())
                .lineToLinearHeading(new Pose2d(forwardAmount, -2, Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> lift(00))
                .build();

        TrajectorySequence threeDotRight = drive.trajectorySequenceBuilder(cycle.end())

                .lineToLinearHeading(new Pose2d(forwardAmount, -26, Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> lift(00))
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
                leftLiftMotor.setPower(-0.3);
                rightLiftMotor.setPower(-0.3);
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