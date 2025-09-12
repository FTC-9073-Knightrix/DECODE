package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.TeleOp.mechanisms.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
public class VisionTesting extends LinearOpMode {
    private MecanumDrive drive = new MecanumDrive(); // importing the drive shit
    private boolean lockOnActive = false;
    private AprilTagDetection lockedTag = null; // lockon value (which one did we lock onto)
    private boolean lastToggle = false;
    private PIDController headingPID = new PIDController(0.03, 0, 0.002); // tune these values

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 400))
                .build();
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            // toggle lockon mode with gamepad1 a
            boolean togglePressed = gamepad1.a;
            if (togglePressed && !lastToggle) {
                lockOnActive = !lockOnActive;
            }
            lastToggle = togglePressed;
            if (!tagProcessor.getDetections().isEmpty()) {
                // find nearest tag (horizontal-plane distance x/z)
                AprilTagDetection tag = tagProcessor.getDetections().get(0); // use first detected tag for now
                telemetry.addData("x", tag.ftcPose.x);
                telemetry.addData("y", tag.ftcPose.y);
                telemetry.addData("z", tag.ftcPose.z);
                telemetry.addData("pitch", tag.ftcPose.pitch);
                telemetry.addData("yaw", tag.ftcPose.yaw);
            }

        }
    }
}
