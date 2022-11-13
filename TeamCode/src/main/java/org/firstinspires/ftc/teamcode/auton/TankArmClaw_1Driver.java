package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Tank Drive - One Driver", group = "LinearOpMode")

public class TankArmClaw_1Driver extends LinearOpMode {
    public DcMotor backLeftMotor = null;
    public DcMotor frontLeftMotor = null;
    public DcMotor backRightMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor liftMotor = null;
    public Servo clawMotor = null;

    static int[] direction = {1, -1};
    static double[] speed = {1.0, 0.25};
    static int[] clawToggle = {0, 1};
    
    public static final int LOW = 1310; //low value here
    public static final int MED = 2050; //med value
    public static final int HI = 2850; //hi value
    public static final int GRD = 00; //ground value
    
    public void runOpMode () {

        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        clawMotor = hardwareMap.get(Servo.class, "clawMotor");

        waitForStart();

        int directionPointer = 0;
        int speedPointer = 0;
        int lift = 0;
        int clawSwitch = 1;
        //liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // assume it is at ground position
        //liftMotor.setPower(1);  // maximum power for movement is 1
        int liftPosition = 0;
        
        while (opModeIsActive()) {
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
            
            telemetry.addData("speedPointer", speedPointer);
            telemetry.addData("directionPointer", directionPointer);
            
            if (gamepad1.dpad_left) {  // strafe left macro
                frontLeftMotor.setPower(-speed[speedPointer]);
                backLeftMotor.setPower(speed[speedPointer]);
                frontRightMotor.setPower(speed[speedPointer]);
                backRightMotor.setPower(-speed[speedPointer]);
            }
            
            else if (gamepad1.dpad_right) {  // strafe right macro
                frontLeftMotor.setPower(speed[speedPointer]);
                backLeftMotor.setPower(-speed[speedPointer]);
                frontRightMotor.setPower(-speed[speedPointer]);
                backRightMotor.setPower(speed[speedPointer]);
            }

            else if (gamepad1.dpad_up) {  // forward macro
                frontLeftMotor.setPower(speed[speedPointer]);
                backLeftMotor.setPower(speed[speedPointer]);
                frontRightMotor.setPower(speed[speedPointer]);
                backRightMotor.setPower(speed[speedPointer]);
            }

            else if (gamepad1.dpad_down) {  // backwards macro
                frontLeftMotor.setPower(-speed[speedPointer]);
                backLeftMotor.setPower(-speed[speedPointer]);
                frontRightMotor.setPower(-speed[speedPointer]);
                backRightMotor.setPower(-speed[speedPointer]);
            }
            
            else{
             
            frontLeftMotor.setPower(speed[speedPointer] * direction[directionPointer] * v3);
            backLeftMotor.setPower(speed[speedPointer] * direction[directionPointer] * v4);
            frontRightMotor.setPower(speed[speedPointer] * direction[directionPointer] * -v2);
            backRightMotor.setPower(speed[speedPointer] * direction[directionPointer] * -v1);
            }
            
            telemetry.addData("frontLeftMotor power", frontLeftMotor.getPower());
            telemetry.addData("backLeftMotor power", backLeftMotor.getPower());
            telemetry.addData("frontRightMotor power", frontRightMotor.getPower());
            telemetry.addData("backRightMotor power", backRightMotor.getPower());

            telemetry.addData("LiftMotorAngle", liftMotor.getCurrentPosition());
            
            
            
            //claw
            
            if (gamepad1.left_trigger >0 && clawSwitch == 0){
                clawSwitch = 1;
            }
            else if (gamepad1.left_bumper) {  // claw open
                clawSwitch = 0;
            }
            clawMotor.setPosition(clawSwitch);
            
            // slow mode
            /*
            if (gamepad1.left_bumper) {
                speedPointer = (speedPointer+1)%2;
            }
            */
            
            

            
            
            /*
            if(gamepad1.y){  // high
                liftMotor.setTargetPosition(HI);
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else if(gamepad1.b){  // medium
                liftMotor.setTargetPosition(MED);
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else if(gamepad1.a){  // low
                liftMotor.setTargetPosition(LOW);
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else if (gamepad1.x && gamepad1.right_trigger > 0) {
                
            }
            */
            
            if(gamepad1.y){  // high
                liftPosition = HI;
                //liftMotor.setTargetPosition(HI);
                //liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else if(gamepad1.b){  // medium
                liftPosition = MED;
                //liftMotor.setTargetPosition(MED);
                //liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else if(gamepad1.a){  // low
                liftPosition = LOW;
                //liftMotor.setTargetPosition(LOW);
                //liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else if (gamepad1.x){
                liftPosition = GRD;
                //liftMotor.setTargetPosition(GRD);
            }
            else if (gamepad1.right_trigger > 0) {  
                liftPosition = liftPosition-30;
            }
            else if (gamepad1.right_bumper) {  
                liftPosition = liftPosition+30;
            }
            
            
                liftMotor.setTargetPosition(liftPosition);
                liftMotor.setPower(1);
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            
            
            telemetry.addData("target position", liftMotor.getTargetPosition());
            telemetry.addData("lift position", liftMotor.getCurrentPosition());
            
                
            
            telemetry.update();
            
        }
    }
}
