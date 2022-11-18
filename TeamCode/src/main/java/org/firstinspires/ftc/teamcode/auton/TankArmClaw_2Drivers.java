//2 drivers
package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name="Tank Drive - Two Drivers", group = "LinearOpMode")

public class TankArmClaw_2Drivers extends LinearOpMode {

    public DcMotor backLeftMotor = null;
    public DcMotor frontLeftMotor = null;
    public DcMotor backRightMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor liftMotor = null;
    public Servo clawMotor = null;

    static final double[] speed = {1.0, 0.15};
    static final double[] toggleDirection = {1.0, -1.0};
//    double clawToggle = 0;
    
    public static final int LOW = 1310; //low value here
    public static final int MED = 2225; //med value
    public static final int HI = 2770; //hi value
    public static final int GRD = 150; //ground value
    
    int speedPointer = 0;
    int directionPointer = 0;
    double clawSwitch = 1;
    int liftPosition = 0;

    @Override
    public void runOpMode(){

        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        //liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        clawMotor = hardwareMap.get(Servo.class, "clawMotor");

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
                liftPosition += 2;
            }
            else if (gamepad2.right_bumper) {  
                liftPosition -= 2;
            }
            
            
                liftMotor.setTargetPosition(liftPosition);
                liftMotor.setPower(1);
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            //claw
            if (gamepad2.left_trigger >0){
                clawSwitch = 1;
            }
            else if (gamepad2.left_bumper) {  // claw open
                clawSwitch = 0;
            }
            
            telemetry.addData("clawSwitch", clawSwitch);
            telemetry.update();
            
            clawMotor.setPosition(clawSwitch);
            
            
            
            
            // slow mode
            if (gamepad1.left_bumper) {
                speedPointer = (speedPointer + 1) % 2;
            }
            
            telemetry.addData("speedPointer", speedPointer);
            telemetry.update();
        }
    }
}