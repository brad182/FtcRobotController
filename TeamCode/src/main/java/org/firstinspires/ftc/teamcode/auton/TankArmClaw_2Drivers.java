//2 drivers
package org.firstinspires.ftc.teamcode.auton;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp (name="Tank Drive - Two Drivers V2", group = "LinearOpMode")
// testtest
public class TankArmClaw_2Drivers extends LinearOpMode {
    public DcMotor backLeftMotor = null;
    public DcMotor frontLeftMotor = null;
    public DcMotor backRightMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor leftLiftMotor = null;
    public DcMotor rightLiftMotor = null;
    public Servo clawMotor = null;
    private Encoder parallelEncoder, perpendicularEncoder;
    static final double[] speed = {1.0, 0.15};
    static final double[] toggleDirection = {1.0, -1.0};
//    double clawToggle = 0;

    public static final int LOW = 1000; //low value here
    public static final int MED = 1630; //med value
    public static final int HI = 2225; //hi value
    public static final int GRD = 000; //ground value

    int speedPointer = 0;
    int directionPointer = 0;
    double clawSwitch = 1;
    int liftPosition = 0;
    private TouchSensor end;
    @Override
    public void runOpMode(){
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        leftLiftMotor = hardwareMap.get(DcMotor.class, "leftLiftMotor");
        rightLiftMotor = hardwareMap.get(DcMotor.class, "rightLiftMotor");
        clawMotor = hardwareMap.get(Servo.class, "clawMotor");
        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "parallelEncoder"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "perpendicularEncoder"));

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

            //macros
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

            //lift buttons (controller 2)
            if(gamepad2.y){  // high
                liftPosition = HI;
                //liftMotor.setTargetPosition(HI);
                //liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else if(gamepad2.b){  // medium
                liftPosition = MED;
                //liftMotor.setTargetPosition(MED);
                //liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else if(gamepad2.a){  // low
                liftPosition = LOW;
                //liftMotor.setTargetPosition(LOW);
                //liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else if (gamepad2.x){
                liftPosition = GRD;
                //liftMotor.setTargetPosition(GRD);
            }
            else if (gamepad2.right_trigger > 0) {
                liftPosition += 4;
            }
            else if (gamepad2.right_bumper) {
                liftPosition -= 4;
            }
            rightLiftMotor.setPower(0.01 * (liftPosition - (rightLiftMotor.getCurrentPosition() + rightLiftMotor.getCurrentPosition()) / 2));
            leftLiftMotor.setPower(0.01 * (liftPosition - (rightLiftMotor.getCurrentPosition() + rightLiftMotor.getCurrentPosition()) / 2));



            //claw
            if (gamepad2.left_trigger >0){
                clawSwitch = 0.93;
            }
            else if (gamepad2.left_bumper) {  // claw open
                clawSwitch = 0;
            }

            telemetry.addData("clawSwitch", clawSwitch);
            clawMotor.setPosition(clawSwitch);



            // slow mode
            if (gamepad1.left_bumper) {
                speedPointer = (speedPointer + 1) % 2;
            }
            //reset lift encoder button
            if(gamepad2.dpad_down){
                leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            //telemetry.addData("speedPointer", speedPointer);
            telemetry.addData("Parallel", parallelEncoder.getCurrentPosition());
            telemetry.addData("Perpendicular", perpendicularEncoder.getCurrentPosition());
            telemetry.update();
        }
    }
}