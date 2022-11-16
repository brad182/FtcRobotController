package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Right Center High", group = "Vision")
public class RightCenterHigh extends LinearOpMode
{
  private BNO055IMU imu;
  private DcMotor frontLeftMotor;
  private DcMotor backLeftMotor;
  private DcMotor backRightMotor;
  private DcMotor frontRightMotor;
  public int CurrentTargetAngle = 0;
  public DcMotor liftMotor = null;
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

    BNO055IMU.Parameters imuParameters;

    imu = hardwareMap.get(BNO055IMU.class, "imu");
    frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
    backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
    backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
    frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
    liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
    clawMotor = hardwareMap.get(Servo.class, "clawMotor");
    backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    // Create new IMU Parameters object.
    imuParameters = new BNO055IMU.Parameters();
    // Use degrees as angle unit.
    imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    // Express acceleration as m/s^2.
    imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    // Disable logging.
    imuParameters.loggingEnabled = false;
    // Initialize IMU.
    imu.initialize(imuParameters);
    // Prompt user to press start buton.
    //telemetry.addData("IMU Example", "Press start to continue...");
    telemetry.update();

    clawMotor.setPosition(1);

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
    if(tagOfInterest != null)
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
      frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
      backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
      backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//      clawMotor.setMode(Servo.RunMode.STOP_AND_RESET_ENCODER);

      clawMotor.setPosition(0);
      sleep(300);
      lift(450);
      sleep(300);
      Forward(35);
      Left(38);
      lift(HI);
      sleep(300);
      Forward(6);
      sleep(300);
      clawMotor.setPosition(0);
      sleep(100);
      lift(GR);
      sleep(300);
      clawMotor.setPosition(1);
      sleep(300);
      Backward(2);

      if (tagOfInterest == null || tagOfInterest.id == LEFT) {
        // pathing for one dot
        telemetry.addLine("One Dot");
        telemetry.update();

        sleep(1000);
        Left(84);
      }
      else if (tagOfInterest.id == MIDDLE) {
        // pathing for two dots
        telemetry.addLine("Two Dots");
        telemetry.update();

        sleep(1000);
        Left(60);
      } else {
        // pathing for three dots
        telemetry.addLine("Three Dots");
        telemetry.update();

        sleep(1000);
        Left(36);
      }
    }

        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending
        while (opModeIsActive()) {sleep(20);}
         */
  }

  void tagToTelemetry(AprilTagDetection detection)
  {
    telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
    telemetry.addLine(String.format("Translation Y: %.2f feet", detection                               .pose.y*FEET_PER_METER));
    telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
    telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
    telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
    telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
  }

  private void ResetEncoders(){
    backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    BNO055IMU.Parameters imuParameters;
    // Create new IMU Parameters object.
    imuParameters = new BNO055IMU.Parameters();
    // Use degrees as angle unit.
    imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    // Express acceleration as m/s^2.
    imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    // Disable logging.
    imuParameters.loggingEnabled = false;
    // Initialize IMU.
    imu.initialize(imuParameters);

  }
  private void Forward(int Inches) {
    Orientation angles;
    double driveangleamount;
    ResetEncoders();
    driveangleamount = 360 * (Inches / (3.141592653589 * 3.8));
    while (backLeftMotor.getCurrentPosition() + backRightMotor.getCurrentPosition() + frontLeftMotor.getCurrentPosition() + frontRightMotor.getCurrentPosition() < driveangleamount * 4) {
      angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
      telemetry.addData("rot about Z", angles.firstAngle);
      backLeftMotor.setPower(0.4 + 0.4 * ((angles.firstAngle) / 10));
      backRightMotor.setPower(0.4 + 0.4 * ((angles.firstAngle) / -10));
      frontLeftMotor.setPower(0.4 + 0.4 * ((angles.firstAngle) / 10));
      frontRightMotor.setPower(0.4 + 0.4 * ((angles.firstAngle) / -10));
      telemetry.update();
    }
    backLeftMotor.setPower(0);
    backRightMotor.setPower(0);
    frontLeftMotor.setPower(0);
    frontRightMotor.setPower(0);
  }
  private void Left(int Inches) {
    Orientation angles;
    double driveangleamount;
    ResetEncoders();

    driveangleamount = 360 * 2 * 0.7 * (Inches / (3.141592653589 * 3.8));
    while (backLeftMotor.getCurrentPosition() - backRightMotor.getCurrentPosition() - frontLeftMotor.getCurrentPosition() + frontRightMotor.getCurrentPosition() < driveangleamount * 4) {
      angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
      telemetry.addData("rot about Z", angles.firstAngle);
      backLeftMotor.setPower(0.4 + 0.4 * ((angles.firstAngle) / 10));
      backRightMotor.setPower(-0.4 + 0.4 * ((angles.firstAngle) / -10));
      frontLeftMotor.setPower(-0.4 + 0.4 * ((angles.firstAngle) / 10));
      frontRightMotor.setPower(0.4 + 0.4 * ((angles.firstAngle) / -10));
      telemetry.update();
    }
    backLeftMotor.setPower(0);
    backRightMotor.setPower(0);
    frontLeftMotor.setPower(0);
    frontRightMotor.setPower(0);
  }
  private void Right(int Inches) {
    Orientation angles;
    double driveangleamount;
    ResetEncoders();

    driveangleamount = 360 * 2 * 0.7 *(Inches / (3.141592653589 * 3.8));
    while (-backLeftMotor.getCurrentPosition() + backRightMotor.getCurrentPosition() + frontLeftMotor.getCurrentPosition() - frontRightMotor.getCurrentPosition() < driveangleamount * 4) {
      angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
      telemetry.addData("rot about Z", angles.firstAngle);
      backLeftMotor.setPower(-0.4 + 0.4 * ((angles.firstAngle) / 10));
      backRightMotor.setPower(0.4 + 0.4 * ((angles.firstAngle) / -10));
      frontLeftMotor.setPower(0.4 + 0.4 * ((angles.firstAngle) / 10));
      frontRightMotor.setPower(-0.4 + 0.4 * ((angles.firstAngle) / -10));
      telemetry.update();
    }
    backLeftMotor.setPower(0);
    backRightMotor.setPower(0);
    frontLeftMotor.setPower(0);
    frontRightMotor.setPower(0);
  }
  private void Backward(int Inches) {
    Orientation angles;
    double driveangleamount;
    ResetEncoders();

    driveangleamount = 360 * (Inches / (3.141592653589 * 3.8));
    while (backLeftMotor.getCurrentPosition() + backRightMotor.getCurrentPosition() + frontLeftMotor.getCurrentPosition() + frontRightMotor.getCurrentPosition() > - driveangleamount * 4) {
      angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
      telemetry.addData("rot about Z", angles.firstAngle);
      backLeftMotor.setPower(-0.4 + 0.4 * ((angles.firstAngle) / 10));
      backRightMotor.setPower(-0.4 + 0.4 * ((angles.firstAngle) / -10));
      frontLeftMotor.setPower(-0.4 + 0.4 * ((angles.firstAngle) / 10));
      frontRightMotor.setPower(-0.4 + 0.4 * ((angles.firstAngle) / -10));
      telemetry.update();
    }
    backLeftMotor.setPower(0);
    backRightMotor.setPower(0);
    frontLeftMotor.setPower(0);
    frontRightMotor.setPower(0);
  }

  private void TurnRight(int TargetAngle) {
    CurrentTargetAngle += TargetAngle;
    Orientation angles;
    double driveangleamount;
    ResetEncoders();
    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    while (angles.firstAngle > -TargetAngle) {
      angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
      telemetry.addData("rot about Z", angles.firstAngle);
      backLeftMotor.setPower(0.4);
      backRightMotor.setPower(-0.4);
      frontLeftMotor.setPower(0.4);
      frontRightMotor.setPower(-0.4);
      telemetry.update();
    }
    backLeftMotor.setPower(0);
    backRightMotor.setPower(0);
    frontLeftMotor.setPower(0);
    frontRightMotor.setPower(0);
  }
  private void TurnLeft(int TargetAngle) {
    CurrentTargetAngle += TargetAngle;
    Orientation angles;
    double driveangleamount;
    ResetEncoders();
    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    while (angles.firstAngle < -TargetAngle) {
      angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
      telemetry.addData("rot about Z", angles.firstAngle);
      backLeftMotor.setPower(-0.4);
      backRightMotor.setPower(0.4);
      frontLeftMotor.setPower(-0.4);
      frontRightMotor.setPower(0.4);
      telemetry.update();
    }
    backLeftMotor.setPower(0);
    backRightMotor.setPower(0);
    frontLeftMotor.setPower(0);
    frontRightMotor.setPower(0);
  }
  private void lift(int height) {  // high
    liftMotor.setTargetPosition(height);
    liftMotor.setPower(1);
    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  }
  private void clawOpen(){
    clawMotor.setPosition(1);
  }
  private void clawClose(){
    clawMotor.setPosition(0);
  }
}

