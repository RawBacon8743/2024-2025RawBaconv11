package org.firstinspires.ftc.teamcode.TeleOp;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="RawBaconTeleop")
//@ServoType(flavor = ServoFlavor.CONTINUOUS)
//@DeviceProperties(xmlTag = "Servo", name = "@string/configTypeServo")


public class RawBaconTeleop extends OpMode {


    DcMotor frontleft;
    DcMotor frontright;
    DcMotor backleft;
    DcMotor backright;
    DcMotor armMotor;
    DcMotor armPivotMotor;
    CRServo leftIntake;
    CRServo rightIntake;
//    Servo leftIntake;

//    Servo rightIntake;

    //    DcMotorEx ArmMotor;
//    DcMotorEx winch;
//    Servo Grabber;
//    Servo Grabber2;
//    Servo droneLauncher;
//    Servo GrabberPivot;
    Double Speed;
    Double Velocity;

    int PivotTargetPostionUp = 10;
    int PivotTargetPostionDown = 1;
    int ArmMotorPosition = 0;
    int PivotMotorPostion = 0;
    //0 is automatic mode 1 is manual mode
    int ArmMode = 1;


    //boolean isrunning;

    @Override
    public void init() {

//        droneLauncher = hardwareMap.get(Servo.class, "launcher");
//        droneLauncher.setPosition(0);



        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");
        armMotor = hardwareMap.get(DcMotor.class, "armmotor");
        leftIntake = hardwareMap.get(CRServo.class, "leftintake");
        rightIntake = hardwareMap.get(CRServo.class, "rightintake");
        armPivotMotor = hardwareMap.get(DcMotor.class, "armpivotmotor");


        frontright.setDirection(DcMotorSimple.Direction.REVERSE);
        backright.setDirection(DcMotorSimple.Direction.REVERSE);
        armPivotMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        winch = hardwareMap.get(DcMotorEx.class, "Winch");
//        ArmMotor = hardwareMap.get(DcMotorEx.class, "ArmMotor");
//        Grabber = hardwareMap.get(Servo.class, "rightgrabber");
//        Grabber2 = hardwareMap.get(Servo.class, "leftgrabber");
//        GrabberPivot = hardwareMap.get(Servo.class, "grabberPivot");


        Speed = 0.6;

        armPivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



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

        //automatic mode
        if (gamepad2.left_bumper) {
            ArmMode = 0;
            armPivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }
        //manual mode
        if (gamepad2.right_bumper){
            ArmMode = 1;
            armPivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }


        //if  (isrunning) {

        if (gamepad1.left_trigger == 1) {
            Speed = 0.2;
        } else if (gamepad1.left_trigger == 0) {
            Speed = 0.6;
        }


        double Pad2LeftStickY = gamepad2.left_stick_y;
        double Pad2RightStickY = -gamepad2.right_stick_y;
        double LeftStickY = gamepad1.left_stick_y;
        double LeftStickX = -gamepad1.left_stick_x;
        double RightStickX = -gamepad1.right_stick_x;


        frontright.setPower((-RightStickX / 1.5 + (LeftStickY - LeftStickX)) * Speed);
        backright.setPower((-RightStickX / 1.5 + (LeftStickY + LeftStickX)) * Speed);
        frontleft.setPower((RightStickX / 1.5 + (LeftStickY + LeftStickX)) * Speed);
        backleft.setPower((RightStickX / 1.5 + (LeftStickY - LeftStickX)) * Speed);

        // intake
        if (gamepad2.left_trigger == 1) {
            leftIntake.setDirection(DcMotorSimple.Direction.FORWARD);
            rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);
            leftIntake.setPower(0.5);
            rightIntake.setPower(0.5);
        } else if (gamepad2.right_trigger == 1) {
            leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
            rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);
            leftIntake.setPower(0.5);
            rightIntake.setPower(0.5);
        } else {
            leftIntake.setPower(0);
            rightIntake.setPower(0);
        }

        if (ArmMode == 0){


           //artemis was not here
            // yes i was
            //nuh uh

            if (gamepad2.left_stick_y != 0)
                armMotor.setPower(gamepad2.left_stick_y / 2);
            else
                armMotor.setPower(0.1);


            if (gamepad2.a){
                armPivotMotor.setTargetPosition(PivotTargetPostionDown);
                armPivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            }

            if (gamepad2.y){
                armPivotMotor.setTargetPosition(PivotTargetPostionUp);
                armPivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            }



        }




        if (ArmMode == 1) {

            if (Pad2RightStickY > 0) {
                armPivotMotor.setPower(Pad2RightStickY / 2);
            }

            if (Pad2RightStickY <= 0) {

                if (gamepad2.dpad_down) {
                    armPivotMotor.setPower(Pad2RightStickY * 0.15);

                } else
                    armPivotMotor.setPower(0.15 + (Pad2RightStickY * 0.13));

            }

            if (gamepad2.left_stick_y != 0)
                armMotor.setPower(gamepad2.left_stick_y / 2);
            else
                armMotor.setPower(0.1);

        }



        ArmMotorPosition = armMotor.getCurrentPosition();
        PivotMotorPostion = armPivotMotor.getCurrentPosition();




        telemetry.addData("ArmMotorPosition: ", ArmMotorPosition);

        telemetry.addData("PivotMotorPostion: ", PivotMotorPostion);

        telemetry.addData("Arm Mode: ", ArmMode);

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