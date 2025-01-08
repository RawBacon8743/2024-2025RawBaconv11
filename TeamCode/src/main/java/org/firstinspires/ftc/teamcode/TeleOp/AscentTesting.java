package org.firstinspires.ftc.teamcode.TeleOp;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="AscentTesting")
//@ServoType(flavor = ServoFlavor.CONTINUOUS)
//@DeviceProperties(xmlTag = "Servo", name = "@string/configTypeServo")

//comment
public class AscentTesting extends OpMode {


    DcMotor frontleft;
    DcMotor frontright;
    DcMotor backleft;
    DcMotor backright;
    DcMotor armMotor;
    DcMotor ascentMotor;
    Servo claw;


//    Servo leftIntake;

//    Servo rightIntake;

    //    DcMotorEx ArmMotor;
//    DcMotorEx winch;
//    Servo Grabber;
//    Servo Grabber2;
//    Servo droneLauncher;
//    Servo GrabberPivot;
    Double WheelSpeed;
    Double Velocity;

    int ArmMotorPosition = 0;
    int AscentMotorPosition = 0;

    boolean Targeting = false;


    //boolean isrunning;

    @Override
    public void init() {

//        droneLauncher = hardwareMap.get(Servo.class, "launcher");
//        droneLauncher.setPosition(0);



        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");
//        armMotor = hardwareMap.get(DcMotor.class, "armmotor");
//        claw = hardwareMap.get(Servo.class, "claw");
        ascentMotor = hardwareMap.get(DcMotor.class, "ascentmotor");


        frontright.setDirection(DcMotorSimple.Direction.REVERSE);
        backright.setDirection(DcMotorSimple.Direction.REVERSE);

//
//        winch = hardwareMap.get(DcMotorEx.class, "Winch");
//        ArmMotor = hardwareMap.get(DcMotorEx.class, "ArmMotor");
//        Grabber = hardwareMap.get(Servo.class, "rightgrabber");
//        Grabber2 = hardwareMap.get(Servo.class, "leftgrabber");
//        GrabberPivot = hardwareMap.get(Servo.class, "grabberPivot");


        WheelSpeed = 0.5;

//        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);



//
//        ArmMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        winch.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }


    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
//        Velocity = 0.0;


    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        //isrunning = true;


    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

//        if (gamepad2.right_trigger == 1 && gamepad2.left_trigger == 1)
//            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//
//
//        //switches bot direction
//
//
//        //if  (isrunning) {
//
//        if (gamepad1.left_trigger == 1) {
//            WheelSpeed = 0.2;
//        } else if (gamepad1.left_trigger == 0) {
//            WheelSpeed = 0.5;
//        }
//
//        double Pad2LeftStickY = gamepad2.left_stick_y;
//        double Pad2RightStickY = -gamepad2.right_stick_y;
//        double LeftStickY = gamepad1.left_stick_y;
//        double LeftStickX = -gamepad1.left_stick_x;
//        double RightStickX = -gamepad1.right_stick_x;
//
//
//        frontright.setPower((-RightStickX / 1.5 + (LeftStickY - LeftStickX)) * WheelSpeed);
//        backright.setPower((-RightStickX / 1.5 + (LeftStickY + LeftStickX)) * WheelSpeed);
//        frontleft.setPower((RightStickX / 1.5 + (LeftStickY + LeftStickX)) * WheelSpeed);
//        backleft.setPower((RightStickX / 1.5 + (LeftStickY - LeftStickX)) * WheelSpeed);
//
//        // intake
//        if (gamepad2.left_bumper){
//            claw.setPosition(0);
//        }
//        else if (gamepad2.right_bumper){
//
//            claw.setPosition(0.35);
//
//        }
//
//
//
//
//        //artemis was not here
//        // yes i was
//        //nuh uh
//
//        if (gamepad2.left_stick_y != 0) {
//            Targeting = false;
//            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            armMotor.setPower(-gamepad2.left_stick_y / 2);
//        }
//        else if (!Targeting)
//            armMotor.setPower(0);
//
//        //picking up is 220
//
//        if (gamepad2.a){
//            Targeting = true;
//            armMotor.setPower(0.5);
//            armMotor.setTargetPosition(250);
//            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
//        //minecraft
//        if (gamepad2.y){
//            Targeting = true;
//            armMotor.setPower(0.5);
//            armMotor.setTargetPosition(1600);
//            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
//
//        if (gamepad2.dpad_down){
//            ascentMotor.setPower(0.1);
//
//        }

        if (gamepad2.right_stick_y != 0) {
            ascentMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ascentMotor.setPower(gamepad2.right_stick_y);
        }

        if (gamepad2.dpad_down){
            ascentMotor.setPower(1);
            ascentMotor.setTargetPosition(50);
            ascentMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (gamepad2.dpad_up){
            ascentMotor.setPower(1);
            ascentMotor.setTargetPosition(3650);
            ascentMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }






        AscentMotorPosition = ascentMotor.getCurrentPosition();



//        telemetry.addData("ArmMotorPosition: ", ArmMotorPosition);
        telemetry.addData("AscentMotorPosition: ", AscentMotorPosition);



        telemetry.update();



//        ArmMotorPosition = armMotor.getCurrentPosition();
//
//        PivotMotorPostion = armPivotMotor.getCurrentPosition();
//
//        telemetry.addData("ArmMotorPosition: ", ArmMotorPosition);
//        telemetry.addData("PivotMotorPostion: ", PivotMotorPostion);
//        telemetry.update();

//        if(gamepad2.left_trigger == 1){
//
//            leftIntake.setPosition(0.6);
//            rightIntake.setPosition(0.4);
//
//        } else if(gamepad2.right_trigger == 1){
//
//            leftIntake.setPosition(0.4);
//            rightIntake.setPosition(0.6);
//
//        } else
//            leftIntake.setPosition(0.5);
//            rightIntake.setPosition(0.5);

//D-PAD STRAFING CODE (NOT WORKING RIGHT)
      /*  if (gamepad1.dpad_left) {
            frontright.setPower(-0.6);
            frontleft.setPower(0.6);
            backleft.setPower(-0.6);
            backright.setPower(0.6);
        } else if (gamepad1.dpad_right) {
            frontright.setPower(0.6);
            frontleft.setPower(-0.6);
            backleft.setPower(0.6);
            backright.setPower(-0.6);
        } else if (gamepad1.dpad_up) {
            frontright.setPower(1);
            frontleft.setPower(1);
            backleft.setPower(1);
            backright.setPower(1);
        } else if (gamepad2.dpad_down) {
            frontright.setPower(-1);
            frontleft.setPower(-1);
            backleft.setPower(-1);
            backright.setPower(-1);
        }*/




//        if (gamepad2.right_bumper) {
//
//            Grabber2.setPosition(0.32);
//
//        } else if (gamepad2.right_trigger == 1) {
//            Grabber2.setPosition(0.205);
//
//        }
//        else if (gamepad2.left_trigger == 1 & gamepad2.left_bumper) {
//            Grabber2.setPosition(0.5);
//
//        } else if (true) {
//            Grabber2.setPosition(0.5);
//        }
//        if (gamepad2.back) {
//            ArmMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//            winch.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        }
//
//        if (gamepad2.a) {
//            ball = 1;
//            ArmMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
////                ArmMotor.setTargetPosition(-150);
////                ArmMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
////                ArmMotor.setVelocity(3000);
//            winch.setTargetPosition(0);
//            // winch.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//            winch.setVelocity(3000);
//
//
//        } else if (gamepad2.b) {
//            ball = 0;
//            winch.setTargetPosition(0);
//            winch.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//            winch.setVelocity(3000);
//
//
//        } else if (gamepad2.x) {
//            ArmMotor.setTargetPosition(-1051);
//            ArmMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//            ArmMotor.setVelocity(750);
//
//        } else if (gamepad2.dpad_up) {
//            ArmMotor.setTargetPosition(-1405);
//            ArmMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//            ArmMotor.setVelocity(750);
//
//        } else if (gamepad2.y) {
//            droneLauncher.setPosition(0.5);
//
//        } else if (!gamepad2.y && !gamepad2.b && !gamepad2.a && !gamepad2.x) {
//            ArmMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//            droneLauncher.setPosition(0);
//
//        }
//        //telemetry.addData("armtick", GrabberPivot.);
//    }
    }
    @Override
    public void stop(){
//        winch.setTargetPosition(0);
//        winch.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        winch.setVelocity(1200);
//
//
//        isrunning = false;
    }
}