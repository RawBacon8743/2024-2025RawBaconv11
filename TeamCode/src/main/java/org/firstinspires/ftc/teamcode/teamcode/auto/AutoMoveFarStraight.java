/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="far straight", group="Linear OpMode")
//@Disabled
public class AutoMoveFarStraight extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor intakeDrive = null;
    //private DcMotorEx leftPod = null;
    //private DcMotorEx rightPod = null;
    //private DcMotorEx perpPod = null;

    //private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    public enum TargetState {
        target1,
        rest1,
        target2,
        rest2
    }

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        intakeDrive = hardwareMap.get(DcMotor.class, "intake");
        //leftPod = hardwareMap.get(DcMotorEx.class, "leftPod");
        //rightPod = hardwareMap.get(DcMotorEx.class, "rightPod");
        //perpPod = hardwareMap.get(DcMotorEx.class, "perpPod");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        intakeDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftDrive. setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive. setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // 1. (Optional) Initialize a vision processor, like the built-in AprilTag engine
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        // 2. Build the Vision Portal using your webcam configuration name
        /*visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor) // Remove or comment out if you are using a blank stream
                .build();

        telemetry.addData("Status", "Camera Initialized. Press Play.");
        telemetry.update();
*/
        waitForStart();
        runtime.reset();

        //Odometry variables
        double x_pos = 0;
        double y_pos = 0;

        double prev_left_encoder_pos = 0;
        double prev_right_encoder_pos = 0;
        double prev_center_encoder_pos = 0;
        double heading = 0;

        final double inchesPerTick = 0.002968435 * 17 / 12 * (8 / 10.91666);//some value please calculate!!
        final double rotationCalibration = 43.2364933897 / 30 * (30 /22.655);
        double trackwidth = 14.28;
        double forward_offset = 4.55;

        final double atanDistStretch = 10;
        final double atanRotStretch = 0.7;

        double target_x_pos   = -24;
        double target_y_pos   = -24;
        double target_heading = Math.toRadians(180);

        double x_Distance_To_Target       = 1000000;
        double y_Distance_To_Target       = 1000000;
        double heading_Distance_To_Target = 1000000;


        double acceptablePosRange = 1;
        double acceptableRotRange = Math.toRadians(3);

        boolean inRange = false;

        double breakTime = 0;

        TargetState targetState = TargetState.target1;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double max;


            // Your loop logic goes here

            // Get the list of all currently detected AprilTags from your processor
//            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
//
//            // Loop through each detection found in the frame
//            for (AprilTagDetection detection : currentDetections) {
//                if (detection instanceof AprilTagSingleDetection) {
//                    AprilTagSingleDetection single = (AprilTagSingleDetection) detection;
//                    int tagId = single.id; // This is the AprilTag ID number
//                    telemetry.addData("Tag ID", tagId);
//                    if (tagId == 33) {
//                        // Do something for Tag 1
//                        x_Distance_To_Target=10;
//                        y_Distance_To_Target=10;
//                        heading_Distance_To_Target=10;
//                }
//
//                // Check if the detection has a valid ID code
//
//
//
//                    // Add logic here for what to do with a specific ID
//
//                }
//            }




            // Odometry code

            //The Targeting Management System exists; It defines the order that the robot should to its steps in.
            inRange = Math.hypot(x_Distance_To_Target, y_Distance_To_Target) < acceptablePosRange & Math.abs(heading_Distance_To_Target) < acceptableRotRange;

            switch (targetState){
                case target1:

                    target_x_pos   = 0;
                    target_y_pos   = 36;
                    target_heading = Math.toRadians(0);


                    if (inRange) {
                        breakTime = runtime.seconds() + 2;
                        targetState = targetState.rest1;
                    }
                    break;
                case rest1:
                    if (runtime.seconds() > breakTime) {
                        targetState = targetState.target2;
                    }
                    break;
                case target2:
                    //target_x_pos   = 0;
                    //target_y_pos   = 0;
                    //target_heading = Math.toRadians(-90); // it took an unoptimal rotation.

                    if (inRange) {
                        breakTime = runtime.seconds() + 2;
                        targetState = targetState.rest2;
                    }

                    break;
                case rest2:
                    telemetry.addData("im in rest",0);

                    break;
            }




            //defines the variables

            double left_encoder_pos   = -frontLeftDrive.getCurrentPosition() * inchesPerTick;
            double right_encoder_pos  =  backLeftDrive.getCurrentPosition() * inchesPerTick;
            double center_encoder_pos =  frontRightDrive.getCurrentPosition() * inchesPerTick;



            // This is the function, it will eventually just be called here and actually defined in another file
            double delta_left_encoder_pos = left_encoder_pos - prev_left_encoder_pos;
            double delta_right_encoder_pos = right_encoder_pos - prev_right_encoder_pos;
            double delta_center_encoder_pos = center_encoder_pos - prev_center_encoder_pos;

            double phi = ((delta_left_encoder_pos - delta_right_encoder_pos) / trackwidth) / rotationCalibration;
            double delta_middle_pos = (delta_left_encoder_pos + delta_right_encoder_pos) / 2;
            double delta_perp_pos = delta_center_encoder_pos - forward_offset * phi;
            double delta_x = -1 * (delta_perp_pos * Math.cos(heading) - delta_middle_pos * Math.sin(heading));
            double delta_y = delta_perp_pos * Math.sin(heading) + delta_middle_pos * Math.cos(heading);

            x_pos   += delta_x;
            y_pos   += delta_y;
            heading += phi;

            prev_left_encoder_pos   = left_encoder_pos;
            prev_right_encoder_pos  = right_encoder_pos;
            prev_center_encoder_pos = center_encoder_pos;


            // goal=range of rotation from-180>180, where 0=0, and something like 270=-90 <- success
            x_Distance_To_Target       = x_pos - target_x_pos;
            y_Distance_To_Target       = y_pos - target_y_pos;
            heading_Distance_To_Target = (heading - target_heading + 180.0) % 360.0 - 180;

            double axial_Distance_To_Target   = -(x_Distance_To_Target * Math.sin(-heading)) + (y_Distance_To_Target * Math.cos(-heading)); //we are working on this math right now, it is not working
            double lateral_Distance_To_Target =  (x_Distance_To_Target * Math.cos(-heading)) + (y_Distance_To_Target * Math.sin(-heading)); //this too


            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -Math.atan(axial_Distance_To_Target/atanDistStretch)*(2/Math.PI);  // Note: pushing stick forward gives negative value
            double lateral = -Math.atan(lateral_Distance_To_Target/atanDistStretch)*(2/Math.PI);
            double yaw     = -Math.atan(heading_Distance_To_Target/atanRotStretch)*(2/Math.PI);

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double frontLeftPower  = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower   = axial - lateral + yaw;
            double backRightPower  = axial + lateral - yaw;
            double intakePower;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            frontLeftPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            backLeftPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            frontRightPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            backRightPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */
            if (gamepad1.right_bumper) {
                intakePower = 1.0;
            } else {
                intakePower = 0.0;
            }


            // Send calculated power to wheels
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);
            intakeDrive.setPower(intakePower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
            telemetry.addData("Intake: ", intakePower);

            telemetry.addData("X position:", x_pos);
            telemetry.addData("Y position:", y_pos);
            telemetry.addData("Heading direction revolutions:" + String.valueOf(Math.floor(heading / (2 * Math.PI))) + "Degrees:", Math.toDegrees(heading % (2 * Math.PI)));

            telemetry.addData("x_Distance_To_Target: ", x_Distance_To_Target);
            telemetry.addData("y_Distance_To_Target: ", y_Distance_To_Target);
            telemetry.addData("Heading_Distance_To_Target: ", heading_Distance_To_Target);

            telemetry.addData("Target State: ", targetState);

            //telemetry.addData("Camera State", visionPortal.getCameraState().toString());

            telemetry.update();
        }
        //visionPortal.close();
    }
}
