package org.firstinspires.ftc.teamcode.teamcode.other;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterMemberMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;

import java.util.ArrayList;
import java.util.List;
@TeleOp(name = "Camera Test OpMode", group = "Concept")
public class webcamTest extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private AprilTagClusterDetection aprilTagClusterDetection;

    private double farness;
    private String ClusterName = "";

    @Override
    public void runOpMode() {
        // 1. (Optional) Initialize a vision processor, like the built-in AprilTag engine
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        //aprilTagProcessor = new AprilTagProcessor.Builder().setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary()).build();

        // 2. Build the Vision Portal using your webcam configuration name
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor) // Remove or comment out if you are using a blank stream
                .build();

        telemetry.addData("Status", "Camera Initialized. Press Play.");
        telemetry.update();

        String Team = "RED";

        waitForStart();

        while (opModeIsActive()) {
            // Your loop logic goes here

            // Get the list of all currently detected AprilTags from your processor
            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();



            // Loop through each detection found in the frame
            for (AprilTagDetection detection : currentDetections) {
                if (detection instanceof AprilTagClusterDetection) {
                    AprilTagClusterDetection single = (AprilTagClusterDetection) detection; //typecasting all detections as single
                    // Check if the detection has a valid ID code <- not applicable
                    //int tagId = single.id; // This is the AprilTag ID number
                    if (detection.ftcPose != null) {
                        farness = detection.ftcPose.y; //This spit out errors
                        telemetry.addData("see tag?", farness);
                        telemetry.addData("Yaw", detection.ftcPose.yaw);
                        ClusterName = ((AprilTagClusterDetection) detection).metadata.name;
                        if (ClusterName.contains(Team)) {
                            telemetry.addLine("I found my cluster.");
                        }
                        telemetry.addLine(ClusterName);


                        //    telemetry.addData("see tag?", check);

                        //telemetry.addData("Tag ID", tagId);

                        // Add logic here for what to do with a specific ID
                        //if (tagId == 33) {
                        //    // Do something for Tag 1
                        //    check = detection.ftcPose.y; //This spit out errors
                        //    telemetry.addData("see tag?", check);
                        //}
                    } else {
                        telemetry.addLine("where is this");
                    }
                }
            }



            // Helpful data output to Driver Station
            telemetry.addData("Camera State", visionPortal.getCameraState().toString() );
            telemetry.update();
        }

        // 3. Clean up and close the portal when the OpMode ends
        visionPortal.close();
    }
}
