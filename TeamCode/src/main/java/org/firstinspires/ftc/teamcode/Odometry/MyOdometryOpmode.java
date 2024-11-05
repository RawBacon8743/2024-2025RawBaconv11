package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.ExpansionHubPIDFVelocityParams;

//import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.util.odometry.OdometryGlobalCoordinatePosition;


/**
 * Created by Sarthak on 10/4/2019.
 */
@TeleOp(name = "My Odometry OpMode")
public class MyOdometryOpmode extends LinearOpMode {
    //Drive motors
    DcMotor right_front, right_back, left_front, left_back;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    final double COUNTS_PER_INCH = 1339.28843739;

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "frontright", rbName = "backright", lfName = "frontleft", lbName = "backleft";
    String verticalLeftEncoderName = rbName, verticalRightEncoderName = lfName, horizontalEncoderName = rfName;

    OdometryGlobalCoordinatePosition globalPositionUpdate;


    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 50);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();


        //positions
        //goToPosition(0*COUNTS_PER_INCH, 24*COUNTS_PER_INCH, 0.5, 0, 0.2*COUNTS_PER_INCH);

        //goToPosition(0*COUNTS_PER_INCH, 0*COUNTS_PER_INCH, 0.5, -180, 0.5*COUNTS_PER_INCH);
        //goToPosition(0*COUNTS_PER_INCH, 0*COUNTS_PER_INCH, 0.5, 180, 0.5*COUNTS_PER_INCH);

        goToPosition(0*COUNTS_PER_INCH, 100*COUNTS_PER_INCH, 0.5, 0, 0.5*COUNTS_PER_INCH);

        goToPosition(0*COUNTS_PER_INCH, 48*COUNTS_PER_INCH, 0.5, 0, 0.5*COUNTS_PER_INCH);
        goToPosition(24*COUNTS_PER_INCH, 48*COUNTS_PER_INCH, 0.5, 0, 0.5*COUNTS_PER_INCH);
        goToPosition(24*COUNTS_PER_INCH, 0*COUNTS_PER_INCH, 0.5, 180, 0.5*COUNTS_PER_INCH);
        goToPosition(48*COUNTS_PER_INCH, 0*COUNTS_PER_INCH, 0.5, 180, 0.5*COUNTS_PER_INCH);
        goToPosition(-48*COUNTS_PER_INCH, 24*COUNTS_PER_INCH, 0.5, 180, 0.5*COUNTS_PER_INCH);
        goToPosition(0*COUNTS_PER_INCH, 0*COUNTS_PER_INCH, 0.5, 0, 0.5*COUNTS_PER_INCH);
        goToPosition(0*COUNTS_PER_INCH, 0*COUNTS_PER_INCH, 0.5, 180, 0.5*COUNTS_PER_INCH);

        while(opModeIsActive()){
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

    //still need to use last 3 variables to set power to motors
    public void goToPosition(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation, double allowableDistanceError){
        double distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();




        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);

        while(opModeIsActive() & distance > allowableDistanceError) {
            distance = Math.hypot(distanceToXTarget, distanceToYTarget);

            distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();

            //double distancein = (distance / COUNTS_PER_INCH) / 20;
            double math = Math.cbrt((Math.abs(distance/(COUNTS_PER_INCH))/20) + 0.2) - 0.37;
            double powervalue = Math.min(math, allowableDistanceError);
            //double math = 2;
            // double powervalue = Math.min(math, 1);




            //if (distance < 3 * COUNTS_PER_INCH) powervalue = 0.4;





            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));

            double robot_movement_x_component = calculateX(robotMovementAngle, robotPower);
            double robot_movement_y_component = calculateY(robotMovementAngle, robotPower);
            double pivotCorrection = (desiredRobotOrientation - globalPositionUpdate.returnOrientation()) * 0.003;




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
        double disterr = allowableDistanceError  / (COUNTS_PER_INCH * 2);
        while(opModeIsActive() & (globalPositionUpdate.returnOrientation() < (desiredRobotOrientation - disterr) || globalPositionUpdate.returnOrientation() > (desiredRobotOrientation + disterr))) {

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

            if(pivotCorrection < 0){
                math1 = Math.cbrt((pivotCorrection/50) - 0.4) + 0.67;
                power = Math.max(math1, -0.5);
            }
            else{
                math1 = Math.cbrt((pivotCorrection/50) + 0.4) - 0.67;
                power = Math.min(math1,0.5);
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

    private void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName){
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
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }
}