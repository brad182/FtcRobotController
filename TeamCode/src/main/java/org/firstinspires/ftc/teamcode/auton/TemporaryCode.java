//2 drivers
package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

//@TeleOp (name="Temporary", group = "LinearOpMode")

public class TemporaryCode extends LinearOpMode {

    public DcMotor backLeftMotor = null;
    public DcMotor frontLeftMotor = null;
    public DcMotor backRightMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor liftMotor = null;
    public CRServo clawMotor = null;

    static final double[] speed = {1.0, 0.15};
    static final double[] toggleDirection = {1.0, -1.0};

    @Override
    public void runOpMode(){

        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        clawMotor = hardwareMap.get(CRServo.class, "clawMotor");
        
        waitForStart();
        int liftPos = 0;
        int speedPointer = 0;
        int directionPointer = 0;
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

            // lift
            telemetry.addData("Lift Value", liftMotor.getCurrentPosition() );
            telemetry.addData("speed", liftMotor.getPower());
            telemetry.addData("liftposvar", liftPos);
            
            
            if(gamepad2.right_trigger > 0){
                liftPos=1;
            } else if (gamepad2.right_bumper){
                liftPos=0;
            } 
            if (liftMotor.getCurrentPosition()<2900 && liftPos == 1){
                liftMotor.setPower(0.05*(2900-liftMotor.getCurrentPosition()));
            }
            else if (liftPos == 0 && liftMotor.getCurrentPosition()>0){
                liftMotor.setPower(-0.3);
            }
            else{
                liftMotor.setPower(0);
            }
            
            //claw
            if (gamepad2.left_trigger > 0){
                clawMotor.setPower(0.5);
            } 
            else if (gamepad2.left_bumper) {
                clawMotor.setPower(-0.5);
            }
            else {
                clawMotor.setPower(0);
            }


            if (gamepad1.x) {
                speedPointer = (speedPointer + 1) % 2;
            }
            
            telemetry.addData("speedPointer", speedPointer);
            telemetry.update();
        }
    }
}