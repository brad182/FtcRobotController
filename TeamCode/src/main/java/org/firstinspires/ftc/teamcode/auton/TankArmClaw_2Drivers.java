//2 drivers
package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp (name="Tank Drive - Two Drivers V2", group = "LinearOpMode")

public class TankArmClaw_2Drivers extends LinearOpMode {
    public DcMotor backLeftMotor = null;
    public DcMotor frontLeftMotor = null;
    public DcMotor backRightMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor leftLiftMotor = null;
    public DcMotor rightLiftMotor = null;
    public Servo clawMotor = null;
    public Servo polePusher = null;
    public Servo perpendicularEncoderLift = null;
    public Servo parallelEncoderLift = null;
    public Encoder parallelEncoder, perpendicularEncoder;
    static final double[] speed = {1.0, 0.15};

    public static final int LOW = 1000; //low value here
    public static final int MED = 1630; //med value
    public static final int HI = 2220; //hi value
    public static final int GRD = 000; //ground value

    public static final int cone2 = 110; //2 cone stack
    public static final int cone3 = 180; //3 cone stack
    public static final int cone4 = 280; //4 cone stack
    public static final int cone5 = 360; //5 cone stack

    int speedPointer = 0;
    double clawSwitch = 0.5;
    int liftPosition = 0;
    double initLiftPosition = 0;
//    private TouchSensor end; //ARE WE STILL USING THIS???
    @Override
    public void runOpMode(){
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        leftLiftMotor = hardwareMap.get(DcMotor.class, "leftLiftMotor");
        rightLiftMotor = hardwareMap.get(DcMotor.class, "rightLiftMotor");
        clawMotor = hardwareMap.get(Servo.class, "clawMotor");
        polePusher = hardwareMap.get(Servo.class, "polePusher");
        perpendicularEncoderLift = hardwareMap.get(Servo.class, "perpendicularEncoderLift");
        parallelEncoderLift = hardwareMap.get(Servo.class, "parallelEncoderLift");
        initLiftPosition = rightLiftMotor.getCurrentPosition();
        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "parallelEncoder"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "perpendicularEncoder"));

//odom wheels retract
        perpendicularEncoderLift.setPosition(0);
        parallelEncoderLift.setPosition(0);

        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while (opModeIsActive()){
            // brad tank drive
            double gamepad1LeftStickX = -gamepad1.left_stick_x;
            double gamepad1LeftStickY = gamepad1.left_stick_y;
            double gamepad1RightStickX = -gamepad1.right_stick_x;
            double gamepad1RightStickY = gamepad1.right_stick_y;
            double r = Math.hypot((4 * gamepad1LeftStickX), gamepad1LeftStickY);
            double r2 = Math.hypot(gamepad1RightStickX, gamepad1RightStickY);
            double robotAngle = Math.atan2(gamepad1LeftStickY, (4 * gamepad1LeftStickX)) - Math.PI / 4;
            double robotAngle2 = Math.atan2(gamepad1RightStickY, gamepad1RightStickX) - Math.PI / 4;
            double rightX = gamepad1RightStickX;
            double leftX = (4 * gamepad1LeftStickX);
            double v3 = r * Math.sin(robotAngle) + leftX;
            double v4 = r * Math.cos(robotAngle) - leftX;
            double v1 = r2 * Math.cos(robotAngle2) + rightX;
            double v2 = r2 * Math.sin(robotAngle2) - rightX;

            frontLeftMotor.setPower(speed[speedPointer] * v3 * 1.3);
            backLeftMotor.setPower(speed[speedPointer] * v4 * 1.3);
            frontRightMotor.setPower(speed[speedPointer] * -v2 * 1.3);
            backRightMotor.setPower(speed[speedPointer] * -v1 * 1.3);

            //drivetrain
            while (gamepad1.dpad_left) {
                frontLeftMotor.setPower(speed[speedPointer]);
                backLeftMotor.setPower(-speed[speedPointer]);
                frontRightMotor.setPower(-speed[speedPointer]);
                backRightMotor.setPower(speed[speedPointer]);
            }
            while (gamepad1.dpad_right) {
                frontLeftMotor.setPower(-speed[speedPointer]);
                backLeftMotor.setPower(speed[speedPointer]);
                frontRightMotor.setPower(speed[speedPointer]);
                backRightMotor.setPower(-speed[speedPointer]);
            }
            while (gamepad1.dpad_up) {
                frontLeftMotor.setPower(-speed[speedPointer]);
                backLeftMotor.setPower(-speed[speedPointer]);
                frontRightMotor.setPower(-speed[speedPointer]);
                backRightMotor.setPower(-speed[speedPointer]);
            }
            while (gamepad1.dpad_down) {
                frontLeftMotor.setPower(speed[speedPointer]);
                backLeftMotor.setPower(speed[speedPointer]);
                frontRightMotor.setPower(speed[speedPointer]);
                backLeftMotor.setPower(speed[speedPointer]);
            }
//junction macros
            if(gamepad2.y && gamepad2.dpad_left != true){  // high
                liftPosition = HI;
                //polePusher.setPosition(0.4);
            }
            else if(gamepad2.b && gamepad2.dpad_left != true){  // medium
                liftPosition = MED;
                //polePusher.setPosition(0.2);
            }
            else if(gamepad2.a && gamepad2.dpad_left != true){  // low
                liftPosition = LOW;
                //polePusher.setPosition(0);
            }
            else if (gamepad2.x && gamepad2.dpad_left != true){ //GRD
                liftPosition = GRD;
                //polePusher.setPosition(0);
            }
            if (rightLiftMotor.getCurrentPosition() > LOW+10){
                polePusher.setPosition(0.2);
            }
            if (rightLiftMotor.getCurrentPosition() > MED+10){
                polePusher.setPosition(0.18);
            }
            if (rightLiftMotor.getCurrentPosition() < LOW - 10){
                polePusher.setPosition(0);
            }



//2-3-4-5 cone stack macros
            if(gamepad2.y && gamepad2.dpad_left){  // 5 cone stack
                liftPosition = cone5;
                //polePusher.setPosition(0);
            }
            else if(gamepad2.b && gamepad2.dpad_left){  // 4 cone stack
                liftPosition = cone4;
                //polePusher.setPosition(0);
            }
            else if(gamepad2.a && gamepad2.dpad_left){  // 3 cone stack
                liftPosition = cone3;
                //polePusher.setPosition(0);
            }
            else if (gamepad2.x && gamepad2.dpad_left){ // 2 cone stack
                liftPosition = cone2;
                //polePusher.setPosition(0);
            }


//manual adjustments
            else if (gamepad2.right_trigger > 0) {
                liftPosition += 4;
            }
            else if (gamepad2.right_bumper) {
                liftPosition -= 4;
            }
            rightLiftMotor.setPower(0.01 * (liftPosition - (rightLiftMotor.getCurrentPosition() + rightLiftMotor.getCurrentPosition() - (initLiftPosition*2) ) / 2));
            leftLiftMotor.setPower(0.01 * (liftPosition - (rightLiftMotor.getCurrentPosition() + rightLiftMotor.getCurrentPosition() - (initLiftPosition*2)) / 2));



            //claw
            if (gamepad2.left_trigger >0){  //open
                clawSwitch = 0.5;
            }
            else if (gamepad2.left_bumper) { //close
                clawSwitch = 0.3;
            }

           // telemetry.addData("clawSwitch", clawSwitch);
            clawMotor.setPosition(clawSwitch);

            // slow mode
            if (gamepad1.left_bumper) {
                speedPointer = (speedPointer + 1) % 2;
            }
            //reset lift encoder button (DO YOU STILL WANT TO TRY TO MAKE THIS WORK??? OR DID U DO SOMETHING ELSE ALREADY HELP IM STUPDI)
//            if(gamepad2.dpad_down){
//                leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            }

            //telemetry.addData("speedPointer", speedPointer);
            //telemetry.addData("Parallel", parallelEncoder.getCurrentPosition());
            //telemetry.addData("Perpendicular", perpendicularEncoder.getCurrentPosition());
            //telemetry.update();

            while (gamepad1.b) {
                frontLeftMotor.setPower(speed[speedPointer]);
                telemetry.addData("front left position : ", frontLeftMotor.getCurrentPosition());
            }
            while (gamepad1.x) {
                backLeftMotor.setPower(speed[speedPointer]);
                telemetry.addData("back left position : ", backLeftMotor.getCurrentPosition());
            }
            while (gamepad1.a) {
                frontRightMotor.setPower(speed[speedPointer]);
                telemetry.addData("front right position : ", frontRightMotor.getCurrentPosition());
            }
            while (gamepad1.y) {
                backRightMotor.setPower(speed[speedPointer]);
                telemetry.addData("back right position : ", backRightMotor.getCurrentPosition());
            }
            telemetry.addData("frontLeft: ", frontLeftMotor.getCurrentPosition());
            telemetry.addData("backLeft: ", backLeftMotor.getCurrentPosition());
            telemetry.addData("frontRight: ", frontRightMotor.getCurrentPosition());
            telemetry.addData("backRight: ", backRightMotor.getCurrentPosition());

            telemetry.update();

        }
    }
}