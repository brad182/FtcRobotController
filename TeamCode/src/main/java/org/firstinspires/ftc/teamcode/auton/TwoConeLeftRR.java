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
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "2 Cone LeftRR", group = "roadrunner")
public class TwoConeLeftRR extends LinearOpMode
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

        Pose2d startPose = new Pose2d(0, 0, 0);

        Trajectory forward1 = drive.trajectoryBuilder(new Pose2d())  // drive forward to pole
                .forward(52)
                .build();

        Trajectory backward2 = drive.trajectoryBuilder(forward1.end()) // back up to get away from vision cone
                .back(2.25)
                .build();

        Trajectory strafeRight3 = drive.trajectoryBuilder(backward2.end())  // align with pole
                .strafeRight(13.35)
                .build();

        Trajectory forward4 = drive.trajectoryBuilder(strafeRight3.end()) // forward to reach the pole
                .forward(5.5)
                .build();

        Trajectory backward5 = drive.trajectoryBuilder(forward4.end()) // back up
                .back(4.75)
                .build();

        Trajectory forward6 = drive.trajectoryBuilder(backward5.end().plus(new Pose2d(0, 0, Math.toRadians(90))), false) // to cone stack
                .forward(38.4)
                .build();

        Trajectory backward7 = drive.trajectoryBuilder(forward6.end()) // back to pole
                .back(39)
                .build();

        Trajectory forward8 = drive.trajectoryBuilder(backward7.end().plus(new Pose2d(0, 0, Math.toRadians(-90))), false) // to pole
                .forward(4.4)
                .build();
        Trajectory backward9 = drive.trajectoryBuilder(forward8.end().plus(new Pose2d(0, 0, Math.toRadians(0))), false) // to pole
                .back(4)
                .build();


        Trajectory oneDotLeft = drive.trajectoryBuilder(backward9.end().plus(new Pose2d(0, 0, Math.toRadians(0))), false)
                .strafeLeft(36)
                .build();

        Trajectory twoDotLeft = drive.trajectoryBuilder(backward9.end().plus(new Pose2d(0, 0, Math.toRadians(0))), false)
                .strafeLeft(12)
                .build();

        Trajectory threeDotRight = drive.trajectoryBuilder(backward9.end().plus(new Pose2d(0, 0, Math.toRadians(0))), false)
                .strafeRight(12)
                .build();
        leftLiftMotor = hardwareMap.get(DcMotor.class, "leftLiftMotor");
        rightLiftMotor = hardwareMap.get(DcMotor.class, "rightLiftMotor");
        clawMotor = hardwareMap.get(Servo.class, "clawMotor");
        perpendicularEncoderLift = hardwareMap.get(Servo.class, "perpendicularEncoderLift");
        parallelEncoderLift = hardwareMap.get(Servo.class, "parallelEncoderLift");
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
            leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            leftLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            // preload

            perpendicularEncoderLift.setPosition(1);
            parallelEncoderLift.setPosition(1);


            clawMotor.setPosition(0);
            //perpendicularEncoderLift.setPosition(0);
            //sleep(1250);
            //lift(450);
            waitFor(1000);
            drive.followTrajectory(forward1);

            drive.followTrajectory(backward2);
            drive.followTrajectory(strafeRight3);
            lift(HI);
            waitFor(1000);
            drive.followTrajectory(forward4);
            waitFor(300); //was 1000
            clawMotor.setPosition(1);  // drop preload
            drive.followTrajectory(backward5);

            // start of cycle -------------------------------------------
            lift(00);
            waitFor(2000);
            lift(215);
            waitFor(1000);
            //lift(90);
            //waitFor(2000);
            //lift(90);
            //waitFor(1000);
            drive.turn(Math.toRadians(90));
            drive.followTrajectory(forward6);  // reach cone stack

            clawMotor.setPosition(0);  // close claw
            waitFor(1000);  // was 1000
            lift(1000);
            waitFor(200);
            drive.followTrajectory(backward7);

            drive.turn(Math.toRadians(-90));  // rotate clockwise to align with pole

            lift(HI);
            waitFor(1000); // was 1000
            drive.followTrajectory(forward8);
            //waitFor(800);
            clawMotor.setPosition(1);  // drop cone
            waitFor(800); // was 800
            drive.followTrajectory(backward5);
            waitFor(500);

            drive.followTrajectory(backward9);

            // end of cycle ----------------------------------------------

            // start of 1 + 2 auton, save for after 1 + 1 is tested and if there is enough time

            /*
            // start of cycle -------------------------------------------
            lift(GR);
            lift(260);
            waitFor(100);
            drive.turn(Math.toRadians(90));
            drive.followTrajectory(forward6);  // reach cone stack

            clawMotor.setPosition(0);  // close claw
            waitFor(300);
            lift(800);
            waitFor(300);
            drive.followTrajectory(backward7);

            drive.turn(Math.toRadians(-90));  // rotate clockwise to align with pole
            lift(HI);
            drive.followTrajectory(forward8);
            waitFor(400);
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
            perpendicularEncoderLift.setPosition(0);
            parallelEncoderLift.setPosition(0);
            lift(00);
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