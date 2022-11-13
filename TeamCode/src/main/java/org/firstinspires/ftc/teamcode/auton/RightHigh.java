package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Right High Junction", group = "Autonomous")
public class RightHigh extends LinearOpMode {

  private BNO055IMU imu;
  private DcMotor frontLeftMotor;
  private DcMotor backLeftMotor;
  private DcMotor backRightMotor;
  private DcMotor frontRightMotor;
  public int CurrentTargetAngle = 0;
  public DcMotor liftMotor = null;
  public Servo clawMotor = null;
  
  static int[] clawToggle = {0, 1};

  public static final int HI = 2850; //hi value
  public static final int GR = 00; //ground value
  
  int clawSwitch = 1;
  int liftPosition = 0;
  String v = "";

  @Override
  public void runOpMode() {
    
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

    waitForStart();


    if (opModeIsActive()) {
      // Put run blocks here.
      frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
      backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
      backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

      clawMotor.setPosition(0);
      sleep(1000);
      lift(450);
      sleep(1500);
      Forward(35);
      sleep(900);
      Right(43);
      lift(HI);
      Forward(6);
      sleep(3000);
      clawMotor.setPosition(1);
      lift(GR);
      lift(250);
      Backward(2);
      
      if(v.equals("red")){
        Right(12);
      }
      else if(v.equals("green")){
        Right(36);
      }
      else if(v.equals("blue")){
        Right(60);
      }

      
      
      }
  }

  /**
  * Describe this function...
  */
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