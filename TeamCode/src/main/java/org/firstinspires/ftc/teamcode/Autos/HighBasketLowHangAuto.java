package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.odometry.OdometryGlobalCoordinatePosition;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name = "HighBasketLowHangAuto")
public class HighBasketLowHangAuto extends LinearOpMode {
    //Drive motors
    DcMotor right_front, right_back, left_front, left_back;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    final double COUNTS_PER_INCH = 336.87;

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "frontright", rbName = "backright", lfName = "frontleft", lbName = "backleft";
    String verticalLeftEncoderName = rbName, verticalRightEncoderName = lfName, horizontalEncoderName = rfName;
    //    DcMotorEx ArmMotor;
//    DcMotorEx winch;
//    Servo Grabber;
//    Servo Grabber2;
    DcMotor frontleft;
    DcMotor frontright;
    DcMotor backleft;
    DcMotor backright;
    DcMotor armPivotMotor;
    CRServo leftIntake;
    CRServo rightIntake;
//    Servo droneLauncher;
//    Servo GrabberPivot;

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    //CV webcam
    OpenCvWebcam webcam1 = null;


    //    @Override
//    public void init() {
//
//        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
//        frontright = hardwareMap.get(DcMotor.class, "frontright");
//        backleft = hardwareMap.get(DcMotor.class, "backleft");
//        backright = hardwareMap.get(DcMotor.class, "backright");
//
//
//        winch = hardwareMap.get(DcMotorEx.class, "Winch");
//        ArmMotor = hardwareMap.get(DcMotorEx.class, "ArmMotor");
//        Grabber = hardwareMap.get(Servo.class, "rightgrabber");
//        Grabber2 = hardwareMap.get(Servo.class, "leftgrabber");
//        droneLauncher = hardwareMap.get(Servo.class, "launcher");
//
//
//        ArmMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//    }




    public void runIntake(String direction, int duration) {

        if (direction == "OUT") {
            leftIntake.setDirection(DcMotorSimple.Direction.FORWARD);
            rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);
            leftIntake.setPower(0.5);
            rightIntake.setPower(0.5);
            sleep(duration);
            leftIntake.setPower(0);
            rightIntake.setPower(0);
        } else if (direction == "IN") {
            leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
            rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);
            leftIntake.setPower(0.5);
            rightIntake.setPower(0.5);
            sleep(duration);
            leftIntake.setPower(0);
            rightIntake.setPower(0);
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

//        winch = hardwareMap.get(DcMotorEx.class, "Winch");
//        ArmMotor = hardwareMap.get(DcMotorEx.class, "ArmMotor");
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");
        armPivotMotor = hardwareMap.get(DcMotor.class, "armpivotmotor");
        leftIntake = hardwareMap.get(CRServo.class, "leftintake");
        rightIntake = hardwareMap.get(CRServo.class, "rightintake");


//        GrabberPivot = hardwareMap.get(Servo.class, "grabberPivot");

//        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        winch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();


        //initializing webcam
//        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
//        //setting webcam to pipeline
//        Pipeline Pipeline = new Pipeline();
//        webcam1.setPipeline(Pipeline);
//
//        //opens camera
//        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                //streams camera to driver hub (640 x 360 in upright position)
//                webcam1.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            //displays error code in case of error
//            @Override
//            public void onError(int errorCode) {
//
//            }
//        });

        waitForStart();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 50);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();


        //positions
        //goToPosition(0*COUNTS_PER_INCH, 24*COUNTS_PER_INCH, 0.5, 0, 0.5*COUNTS_PER_INCH);

        //goToPosition(0*COUNTS_PER_INCH, 0*COUNTS_PER_INCH, 0.5, -180, 0.5*COUNTS_PER_INCH);
        //goToPosition(0*COUNTS_PER_INCH, 0*COUNTS_PER_INCH, 0.5, 180, 0.5*COUNTS_PER_INCH);

       /* goToPosition(0*COUNTS_PER_INCH, 100*COUNTS_PER_INCH, 0.5, 0, 0.5*COUNTS_PER_INCH);

        goToPosition(0*COUNTS_PER_INCH, 48*COUNTS_PER_INCH, 0.5, 0, 0.5*COUNTS_PER_INCH);
        goToPosition(24*COUNTS_PER_INCH, 48*COUNTS_PER_INCH, 0.5, 0, 0.5*COUNTS_PER_INCH);
        goToPosition(24*COUNTS_PER_INCH, 0*COUNTS_PER_INCH, 0.5, 180, 0.5*COUNTS_PER_INCH);
        goToPosition(48*COUNTS_PER_INCH, 0*COUNTS_PER_INCH, 0.5, 180, 0.5*COUNTS_PER_INCH);
        goToPosition(-48*COUNTS_PER_INCH, 24*COUNTS_PER_INCH, 0.5, 180, 0.5*COUNTS_PER_INCH);
        goToPosition(0*COUNTS_PER_INCH, 0*COUNTS_PER_INCH, 0.5, 0, 0.5*COUNTS_PER_INCH);
        goToPosition(0*COUNTS_PER_INCH, 0*COUNTS_PER_INCH, 0.5, 180, 0.5*COUNTS_PER_INCH); */

        //LeftRed
//        sleep(1000);


        //move to high basket

        goToPosition(-16 *COUNTS_PER_INCH, 14 * COUNTS_PER_INCH,0.5,0,0.5 * COUNTS_PER_INCH);

        goToPosition(-16 *COUNTS_PER_INCH, 14 * COUNTS_PER_INCH,0.5,-125,0.5 * COUNTS_PER_INCH);

        //put into high basket

        //move to submersible

        sleep(3000);

        goToPosition(-16 *COUNTS_PER_INCH, 14 * COUNTS_PER_INCH,0.5,0,0.5 * COUNTS_PER_INCH);

        goToPosition(6 *COUNTS_PER_INCH, 52 * COUNTS_PER_INCH,0.5,0,0.5 * COUNTS_PER_INCH);

        goToPosition(6 *COUNTS_PER_INCH, 52 * COUNTS_PER_INCH,0.5,90,0.5 * COUNTS_PER_INCH);


//        goToPosition(-6 * COUNTS_PER_INCH, 10 * COUNTS_PER_INCH, 0.5, 240, 0.5 * COUNTS_PER_INCH);
//        sleep(500);
//        goToPosition(-6 * COUNTS_PER_INCH, 10 * COUNTS_PER_INCH, 0.5, 0, 0.5 * COUNTS_PER_INCH);
//        sleep(500);
//        goToPosition(0 * COUNTS_PER_INCH, 60 * COUNTS_PER_INCH, 0.5, 0, 0.5 * COUNTS_PER_INCH);
//        sleep(500);
//        goToPosition(-9 * COUNTS_PER_INCH, 60 * COUNTS_PER_INCH, 0.5, 0, 0.5 * COUNTS_PER_INCH);
//        sleep(500);
//        goToPosition(-9 * COUNTS_PER_INCH, 60 * COUNTS_PER_INCH, 0.5, 180, 0.5 * COUNTS_PER_INCH);
//        sleep(500);
//        goToPosition(-12 * COUNTS_PER_INCH, 10 * COUNTS_PER_INCH, 0.5, 180, 0.5 * COUNTS_PER_INCH);


//        sleep(5000);
//        frontright.setDirection(DcMotorSimple.Direction.REVERSE);
//        backright.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
//        backleft.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontleft.setPower(0.2);
//        frontright.setPower(0.2);
//        backleft.setPower(0.2);
//        backright.setPower(0.2);
//        sleep(1800);
//        frontright.setDirection(DcMotorSimple.Direction.FORWARD);
//        backright.setDirection(DcMotorSimple.Direction.FORWARD);
//        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
//        backleft.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontleft.setPower(0.2);
//        frontright.setPower(0.2);
//        backleft.setPower(0.2);
//        backright.setPower(0.2);
//        sleep(4500);
//        frontleft.setPower(0);
//        frontright.setPower(0);
//        backleft.setPower(0);
//        backright.setPower(0);
        //super cool coding

//        goToPosition(48*COUNTS_PER_INCH, 0*COUNTS_PER_INCH, 0.2, 0, 0.5*COUNTS_PER_INCH);




        sleep(1000);
//        int type = Pipeline.getAutoType();
        switch (0){

            case 0:
                //bumper = closed
                //trigger = open
                //no -1500
//                winch.setTargetPosition(-80);
//                winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                winch.setVelocity(2500);
//                ArmMotor.setTargetPosition(-20);
//                ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                ArmMotor.setVelocity(750);
//                //raises grabber slightly as to not bump up agaist ground
//
//                goToPosition(0 * COUNTS_PER_INCH, 35 * COUNTS_PER_INCH, 0.5, 0, 0.5 * COUNTS_PER_INCH);
//                goToPosition(2 * COUNTS_PER_INCH, 35* COUNTS_PER_INCH, 0.5, -90, 0.5 * COUNTS_PER_INCH);
//                //Both lines above strafe into the spiked tape with the pixel underneath
//
//                Grabber2.setPosition(0.205);
//
//                //Line above drops the pixel
//
//                sleep(300);
//                goToPosition(6 * COUNTS_PER_INCH, 35 * COUNTS_PER_INCH, 0.5, -90, 0.5 * COUNTS_PER_INCH);
//                //Line above moves back
//                Grabber2.setPosition(0.32);
//
//                sleep(300);
//                goToPosition(6 * COUNTS_PER_INCH, 55 * COUNTS_PER_INCH, 0.5, -90, 0.5 * COUNTS_PER_INCH);
//                //Line above moves towards the center
//
//                sleep(300);
//                goToPosition(6 * COUNTS_PER_INCH, 55 * COUNTS_PER_INCH, 0.5, 91, 0.5 * COUNTS_PER_INCH);
//                //Line above rotates 180
//
//                sleep(300);
//                goToPosition(55 * COUNTS_PER_INCH, 55 * COUNTS_PER_INCH, 0.5, 91, 0.5 * COUNTS_PER_INCH);
//                sleep(300);
//                goToPosition(86 * COUNTS_PER_INCH, 27 * COUNTS_PER_INCH, 0.5, 92, 0.5 * COUNTS_PER_INCH);
//                //Lines above moves towards the board
//
//                sleep(300);
//                winch.setTargetPosition(-4200);
//                winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                winch.setVelocity(2500);
//                ArmMotor.setTargetPosition(-750);
//                ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                ArmMotor.setVelocity(750);
//                sleep(500);
//                GrabberPivot.setPosition(1);
//                sleep(1000);
//                //arm, winch, and pivot goes up
//
//                frontleft.setPower(.2);
//                frontright.setPower(.2);
//                backleft.setPower(.2);
//                backright.setPower(.2);
//                sleep(1000);
//                frontleft.setPower(0);
//                frontright.setPower(0);
//                backleft.setPower(0);
//                backright.setPower(0);
//                sleep(300);
//                Grabber.setPosition(0.26);
//                //bumps up to board and drops pixel
//
//                sleep(300);
//                goToPosition(84 * COUNTS_PER_INCH, 28 * COUNTS_PER_INCH, 0.5, 92, 0.5 * COUNTS_PER_INCH);
//                goToPosition(84 * COUNTS_PER_INCH, 45 * COUNTS_PER_INCH, 0.5, 92, 0.5 * COUNTS_PER_INCH);
//                sleep(300);
//                goToPosition(90 * COUNTS_PER_INCH, 45 * COUNTS_PER_INCH, 0.5, 92, 0.5 * COUNTS_PER_INCH);
//                //Lines above park on left side corner
//                sleep(300);
//
//                Grabber.setPosition(0.14);
//                Grabber2.setPosition(0.32);
//                GrabberPivot.setPosition(0.77);
//                ArmMotor.setTargetPosition(0);
//                ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                ArmMotor.setVelocity(750);
//                winch.setTargetPosition(0);
//                winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                winch.setVelocity(2500);
//                //lowers arm winch pivot safely
//
//                break;

            case 1:
//                //working
//                winch.setTargetPosition(-80);
//                winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                winch.setVelocity(2500);
//                ArmMotor.setTargetPosition(-20);
//                ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                ArmMotor.setVelocity(750);
//                //raises grabber
//
//                goToPosition(0 * COUNTS_PER_INCH, 25 * COUNTS_PER_INCH, 0.5, 0, 0.5 * COUNTS_PER_INCH);
//                goToPosition(0 * COUNTS_PER_INCH, 25 * COUNTS_PER_INCH, 0.5, 180, 0.5 * COUNTS_PER_INCH);
//                goToPosition(4 * COUNTS_PER_INCH, 50 * COUNTS_PER_INCH, 0.5, 180, 0.5 * COUNTS_PER_INCH);
//                Grabber2.setPosition(0.205);
//
//                //lines up spiketape and drops pixel
//
//                sleep(300);
//                goToPosition(4 * COUNTS_PER_INCH, 55 * COUNTS_PER_INCH, 0.5, 180, 0.5 * COUNTS_PER_INCH);
//                Grabber2.setPosition(0.32);
//                goToPosition(4 * COUNTS_PER_INCH, 55 * COUNTS_PER_INCH, 0.5, 90, 0.5 * COUNTS_PER_INCH);
//                goToPosition(85 * COUNTS_PER_INCH, 55 * COUNTS_PER_INCH, 0.5, 90, 0.5 * COUNTS_PER_INCH);
//                goToPosition(85 * COUNTS_PER_INCH, 17 * COUNTS_PER_INCH, 0.5, 90, 0.5 * COUNTS_PER_INCH);
//                //lines up for board
//
//                sleep(300);
//                winch.setTargetPosition(-4200);
//                winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                winch.setVelocity(2500);
//                ArmMotor.setTargetPosition(-750);
//                ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                ArmMotor.setVelocity(750);
//                sleep(500);
//                GrabberPivot.setPosition(1);
//                sleep(1000);
//                //arm, winch, and pivot go up
//
//                frontright.setPower(0.2);
//                backright.setPower(0.2);
//                frontleft.setPower(0.2);
//                backleft.setPower(0.2);
//                sleep(1000);
//                frontright.setPower(0);
//                backright.setPower(0);
//                frontleft.setPower(0);
//                backleft.setPower(0);
//                sleep(300);
//                Grabber.setPosition(0.26);
//                //bumps up on board and drops pixel
//
//                sleep(300);
//                goToPosition(84 * COUNTS_PER_INCH, 17 * COUNTS_PER_INCH, 0.5, 92, 0.5 * COUNTS_PER_INCH);
//                goToPosition(84 * COUNTS_PER_INCH, 45 * COUNTS_PER_INCH, 0.5, 92, 0.5 * COUNTS_PER_INCH);
//                sleep(300);
//                goToPosition(90 * COUNTS_PER_INCH, 45 * COUNTS_PER_INCH, 0.5, 92, 0.5 * COUNTS_PER_INCH);
//                //Lines above park on left side corner
//                sleep(300);
//
//                Grabber.setPosition(0.14);
//                Grabber2.setPosition(0.32);
//                GrabberPivot.setPosition(0.77);
//                ArmMotor.setTargetPosition(0);
//                ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                ArmMotor.setVelocity(750);
//                winch.setTargetPosition(0);
//                winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                winch.setVelocity(2500);
//                //lowers arm winch pivot safely
//                break;

            case 2:
//                winch.setTargetPosition(-80);
//                winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                winch.setVelocity(2500);
//                ArmMotor.setTargetPosition(-20);
//                ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                ArmMotor.setVelocity(750);
//                //raises arm
//
//                goToPosition(0 * COUNTS_PER_INCH, 15 * COUNTS_PER_INCH, 0.5, 0, 0.5 * COUNTS_PER_INCH);
//                goToPosition(0 * COUNTS_PER_INCH, 35 * COUNTS_PER_INCH, 0.5, 0, 0.5 * COUNTS_PER_INCH);
//                sleep(300);
//                goToPosition(0 * COUNTS_PER_INCH, 35 * COUNTS_PER_INCH, 0.5, 91, 0.5 * COUNTS_PER_INCH);
//                //Both lines above strafe into the spiked tape with the pixel underneath
//                sleep(300);
//                goToPosition(6 * COUNTS_PER_INCH, 35* COUNTS_PER_INCH, 0.5, 91, 0.5 * COUNTS_PER_INCH);
//                //Line above moves forward
//                Grabber2.setPosition(0.205);
//                //Line above drops the pixel
//                sleep(300);
//                goToPosition(-5 * COUNTS_PER_INCH, 35 * COUNTS_PER_INCH, 0.5, 91, 0.5 * COUNTS_PER_INCH);
//                //Line above moves back
//                Grabber2.setPosition(0.32);
//                sleep(300);
//                goToPosition(-5 * COUNTS_PER_INCH, 55 * COUNTS_PER_INCH, 0.5, 91, 0.5 * COUNTS_PER_INCH);
//                //Line above moves towards the center
//                goToPosition(65 * COUNTS_PER_INCH, 55 * COUNTS_PER_INCH, 0.5, 92, 0.5 * COUNTS_PER_INCH);
//                goToPosition(84 * COUNTS_PER_INCH, 9 * COUNTS_PER_INCH, 0.5, 92, 0.5 * COUNTS_PER_INCH);
//                //Lines above moves towards the board
//                sleep(1000);
//
//                sleep(300);
//                winch.setTargetPosition(-4200);
//                winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                winch.setVelocity(2500);
//                ArmMotor.setTargetPosition(-750);
//                ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                ArmMotor.setVelocity(750);
//                sleep(500);
//                GrabberPivot.setPosition(1);
//                sleep(500);
//                //arm, winch, and pivot go up
//
//                frontright.setPower(0.2);
//                backright.setPower(0.2);
//                frontleft.setPower(0.2);
//                backleft.setPower(0.2);
//                sleep(1000);
//                frontright.setPower(0);
//                backright.setPower(0);
//                frontleft.setPower(0);
//                backleft.setPower(0);
//                sleep(300);
//                Grabber.setPosition(0.26);
//                //bumps up and drops pixel
//
//
//                sleep(300);
//                goToPosition(84 * COUNTS_PER_INCH, 9 * COUNTS_PER_INCH, 0.5, 92, 0.5 * COUNTS_PER_INCH);
//                goToPosition(84 * COUNTS_PER_INCH, 30 * COUNTS_PER_INCH, 0.5, 92, 0.5 * COUNTS_PER_INCH);
//                goToPosition(90 * COUNTS_PER_INCH, 45 * COUNTS_PER_INCH, 0.5, 92, 0.5 * COUNTS_PER_INCH);
//                //Lines above park on left side corner
//                sleep(300);
//
//                Grabber.setPosition(0.14);
//                Grabber2.setPosition(0.32);
//                GrabberPivot.setPosition(0.77);
//                ArmMotor.setTargetPosition(0);
//                ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                ArmMotor.setVelocity(750);
//                winch.setTargetPosition(0);
//                winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                winch.setVelocity(2500);
//                //lowers everything safely
//                break;
//
//            default:
//                System.out.println("default");
//                break;
        }

//

        while (opModeIsActive()) {
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());

            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();

        }

        //Stop the thread
        globalPositionUpdate.stop();

    }

    class Pipeline extends OpenCvPipeline {
        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat midCrop;
        Mat rightCrop;
        double leftavgfin;
        double midavgfin;
        double rightavgfin;
        Mat outPut = new Mat();
        Scalar rectColor = new Scalar(255.0, 0.0, 0.0);

        public int autoType;

        public int getAutoType(){
            return autoType;
        }

        /*
         * remember, whenever you get an implement methods error or
         * they import a random method, use @Override because you are
         * modifying an existing method
         */

        @Override
        //input is the frame the camera sees
        //processFrame runs continuously
        public Mat processFrame(Mat input) {

            //Converts from RGB to YCbCr
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);


            //creates left, middle, and right detection areas
            Rect leftRect = new Rect(400,500,300,200);
            Rect midRect = new Rect(950, 450, 400, 200);
            Rect rightRect = new Rect(1600,500,300,200);


            //creates bounding boxes for rectangles
            //(can't see them on driver's station otherwise)
            input.copyTo(outPut);
            Imgproc.rectangle(outPut, leftRect, rectColor, 2);
            Imgproc.rectangle(outPut, midRect, rectColor, 2);
            Imgproc.rectangle(outPut, rightRect, rectColor, 2);

            //Creates submats (crops of camera frame) from our rectangles
            leftCrop = YCbCr.submat(leftRect);
            midCrop = YCbCr.submat(midRect);
            rightCrop = YCbCr.submat(rightRect);

            //extracts red channel from YCbCr
            Core.extractChannel(leftCrop, leftCrop, 1);
            Core.extractChannel(midCrop, midCrop, 1);
            Core.extractChannel(rightCrop, rightCrop, 1);

            //Creates averages from each crop
            //Higher average means more of the color you're detecting
            Scalar leftavg = Core.mean(leftCrop);
            Scalar midavg = Core.mean(midCrop);
            Scalar rightavg = Core.mean(rightCrop);

            //Converts stinky scalars to nice and usable doubles
            //scalar is basically an array, we want the first element "val[0}"
            leftavgfin = leftavg.val[0];
            midavgfin = midavg.val[0];
            rightavgfin = rightavg.val[0];

            /* Red side (CV1 and CV2) will take the MAXIMUM average
             while Blue side (CV3 and CV4) will take the MINIMUM average
             this is to change between detecting red or blue
            */

            //if the left side has the most red
            if (Math.max(leftavgfin, midavgfin) == leftavgfin && Math.max(leftavgfin, rightavgfin) == leftavgfin){
                //Prints left to driver hub
                System.out.println("left");
                //Directs Autonomous

                autoType = 0;
            }
            //if the middle has the most red
            if (Math.max(midavgfin, leftavgfin) == midavgfin && Math.max(midavgfin, rightavgfin) == midavgfin){
                //Prints Middle to driver hub
                System.out.println("middle");
                //Directs Autonomous

                autoType = 1;
            }
            //if the right has the most red
            if (Math.max(rightavgfin, leftavgfin) == rightavgfin && Math.max(rightavgfin, midavgfin) == rightavgfin){
                //Prints Right to driver hub
                System.out.println("right");
                //Directs Autonomous

                autoType = 2;
            }

            return outPut;
        }

    }

    //still need to use last 3 variables to set power to motors
    public void goToPosition(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation, double allowableDistanceError) {
        double distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();


        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);

        while (opModeIsActive() & distance > allowableDistanceError) {
            distance = Math.hypot(distanceToXTarget, distanceToYTarget);

            distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();

            //double distancein = (distance / COUNTS_PER_INCH) / 20;
            double math = Math.cbrt((Math.abs(distance / (COUNTS_PER_INCH)) / 20) + 0.2) - 0.37;
            double powervalue = Math.min(math, allowableDistanceError);
            //double math = 2;
            // double powervalue = Math.min(math, 1);


            //if (distance < 3 * COUNTS_PER_INCH) powervalue = 0.4;


            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));

            double robot_movement_x_component = calculateX(robotMovementAngle, robotPower);
            double robot_movement_y_component = calculateY(robotMovementAngle, robotPower);
            double pivotCorrection = (desiredRobotOrientation - globalPositionUpdate.returnOrientation()) * 0.004;


            double x_rotated = (robot_movement_x_component * Math.cos(Math.toRadians(globalPositionUpdate.returnOrientation())) - robot_movement_y_component * Math.sin(Math.toRadians(globalPositionUpdate.returnOrientation()))) * 1.3;
            double y_rotated = robot_movement_x_component * Math.sin(Math.toRadians(globalPositionUpdate.returnOrientation())) + robot_movement_y_component * Math.cos(Math.toRadians(globalPositionUpdate.returnOrientation()));

            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());




            /*
            telemetry.addData("istan", distance/COUNTS_PER_INCH);
            telemetry.addData("powervalue", powervalue);
            telemetry.addData("leftback motor", left_back.getPower());
            telemetry.addData("leftfront motor", left_front.getPower());
            telemetry.addData("rightback motor", right_back.getPower());
            telemetry.addData("rightfront motor", right_front.getPower());
            */
            double denominator = Math.max(Math.abs(y_rotated) + Math.abs(x_rotated) + Math.abs(pivotCorrection), 1);
            // if not working take Math.abs of powervalue
            left_back.setPower((pivotCorrection) + ((y_rotated - x_rotated) / denominator) * powervalue);
            left_front.setPower((pivotCorrection) + ((y_rotated + x_rotated) / denominator) * powervalue);
            right_back.setPower((-pivotCorrection) + ((y_rotated + x_rotated) / denominator) * powervalue);
            right_front.setPower((-pivotCorrection) + ((y_rotated - x_rotated) / denominator) * powervalue);

            telemetry.update();
        }
        double disterr = allowableDistanceError / (COUNTS_PER_INCH * 2);
        while (opModeIsActive() & (globalPositionUpdate.returnOrientation() < (desiredRobotOrientation - disterr) || globalPositionUpdate.returnOrientation() > (desiredRobotOrientation + disterr))) {

            double pivotCorrection = (desiredRobotOrientation - globalPositionUpdate.returnOrientation());


            /*
            distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();

            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));

            double robot_movement_x_component = calculateX(robotMovementAngle, robotPower);
            double robot_movement_y_component = calculateY(robotMovementAngle, robotPower);

            double x_rotated = (robot_movement_x_component * Math.cos(Math.toRadians(globalPositionUpdate.returnOrientation()) - robot_movement_y_component * Math.sin(Math.toRadians(globalPositionUpdate.returnOrientation()))) * 1.3)/5;
            double y_rotated = (robot_movement_x_component * Math.sin(Math.toRadians(globalPositionUpdate.returnOrientation())) + robot_movement_y_component * Math.cos(Math.toRadians(globalPositionUpdate.returnOrientation())))/5;
            */


            double math1 = 1;
            double power = 1;

            if (pivotCorrection < 0) {
                math1 = Math.cbrt((pivotCorrection / 50) - 0.4) + 0.67;
                power = Math.max(math1, -0.5);
            } else {
                math1 = Math.cbrt((pivotCorrection / 50) + 0.4) - 0.67;
                power = Math.min(math1, 0.5);
            }




            /*
            if(pivotCorrection < 0){
                power = Math.cbrt(pivotCorrection - 0.2) + 0.37;
            }
            else{
                power = Math.cbrt(pivotCorrection + 0.2) - 0.37;
            }*/
            //double power = Math.log(Math.abs(pivotCorrection + 2)) / Math.log(10);
            /*
            double denominator = Math.max(Math.abs(x_rotated) + Math.abs(y_rotated) + Math.abs(power), 1);

            left_back.setPower((power) + ((y_rotated - x_rotated) / denominator));
            left_front.setPower((power) + ((y_rotated + x_rotated) / denominator));
            right_back.setPower((-power) + ((y_rotated + x_rotated) / denominator));
            right_front.setPower((-power) + ((y_rotated - x_rotated) / denominator));
            */


            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());


            left_back.setPower(power);
            left_front.setPower(power);
            right_back.setPower(-power);
            right_front.setPower(-power);


            /*
            telemetry.addData("rpow", robotPower);
            telemetry.addData("botangle", globalPositionUpdate.returnOrientation());
            telemetry.addData("allowabledist", allowableDistanceError / COUNTS_PER_INCH);
            telemetry.addData("pivot", pivotCorrection);
            telemetry.addData("power!!!", power);
            telemetry.addData("math1",math1);
            */
            telemetry.update();

        }
        left_back.setPower(0);
        left_front.setPower(0);
        right_front.setPower(0);
        right_back.setPower(0);
    }

    private void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName) {
        right_front = hardwareMap.dcMotor.get(rfName);
        right_back = hardwareMap.dcMotor.get(rbName);
        left_front = hardwareMap.dcMotor.get(lfName);
        left_back = hardwareMap.dcMotor.get(lbName);

        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


//        droneLauncher = hardwareMap.get(Servo.class, "launcher");
//        droneLauncher.setPosition(0);
//
//        Grabber = hardwareMap.get(Servo.class, "rightgrabber");
//        Grabber2 = hardwareMap.get(Servo.class, "leftgrabber");
//        GrabberPivot = hardwareMap.get(Servo.class, "grabberPivot");
//
//        Grabber.setPosition(0.14);
//        Grabber2.setPosition(0.32);
//        GrabberPivot.setPosition(0.77);

        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        //right_front.setDirection(DcMotorSimple.Direction.REVERSE);
        //right_back.setDirection(DcMotorSimple.Direction.REVERSE);
        left_back.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }

    /**
     * Calculate the power in the x direction
     *
     * @param desiredAngle angle on the x axis
     * @param speed        robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     *
     * @param desiredAngle angle on the y axis
     * @param speed        robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }
}