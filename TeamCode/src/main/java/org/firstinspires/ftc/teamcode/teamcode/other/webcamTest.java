package org.firstinspires.ftc.teamcode.teamcode.other;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;

import java.util.ArrayList;
import java.util.List;
@TeleOp(name = "Camera Test OpMode", group = "Concept")
public class webcamTest extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private AprilTagSingleDetection aprilTagSingleDetection;

    private double check;

    @Override
    public void runOpMode() {
        // 1. (Optional) Initialize a vision processor, like the built-in AprilTag engine
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        aprilTagProcessor = new AprilTagProcessor.Builder().setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary()).build();

        // 2. Build the Vision Portal using your webcam configuration name
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor) // Remove or comment out if you are using a blank stream
                .build();

        telemetry.addData("Status", "Camera Initialized. Press Play.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Your loop logic goes here

            // Get the list of all currently detected AprilTags from your processor
            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();



            // Loop through each detection found in the frame
            for (AprilTagDetection detection : currentDetections) {
                if (detection instanceof AprilTagSingleDetection) {
                    AprilTagSingleDetection single = (AprilTagSingleDetection) detection; //typecasting all detections as single
                    // Check if the detection has a valid ID code
                    int tagId = single.id; // This is the AprilTag ID number
                    telemetry.addLine("yo");
                    if (detection.ftcPose != null) {
                        telemetry.addData("Tag ID", tagId);

                        // Add logic here for what to do with a specific ID
                        if (tagId == 33) {
                            // Do something for Tag 1
                            check = detection.ftcPose.y; //This spit out errors
                            telemetry.addData("see tag?", check);
                        }
                    } else {
                        telemetry.addLine("where is this");
                    }
                }
            }
            telemetry.update();


            // Helpful data output to Driver Station
            telemetry.addData("Camera State", visionPortal.getCameraState().toString() );
            telemetry.addLine("hey");
            telemetry.update();
        }

        // 3. Clean up and close the portal when the OpMode ends
        visionPortal.close();
    }
}
