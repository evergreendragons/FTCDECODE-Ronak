package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name = "Show AprilTag Distance Only", group = "Vision")
public class ShowAprilTagDistanceOnly extends LinearOpMode {

    private Limelight3A limelight;

    // Constants for distance calculation
    private static final double TAG_HEIGHT_INCHES = 60.0;   // height of AprilTag center from floor
    private static final double CAM_HEIGHT_INCHES = 20.0;   // height of Limelight from floor
    private static final double CAM_ANGLE_DEGREES = 25.0;   // Limelight tilt angle (upward)

    private static final int TARGET_ID = 20;  // AprilTag ID to track (adjust as needed)

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Fast update rate for telemetry and polling
        telemetry.setMsTransmissionInterval(11);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(4);  // Use your AprilTag pipeline
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                List<FiducialResult> tags = result.getFiducialResults();
                for (FiducialResult tag : tags) {
                    if (tag.getFiducialId() == TARGET_ID) {
                        double ty = tag.getTargetYDegrees();  // vertical offset
                        double totalAngleDeg = CAM_ANGLE_DEGREES + ty;
                        double totalAngleRad = Math.toRadians(totalAngleDeg);

                        double distance = (TAG_HEIGHT_INCHES - CAM_HEIGHT_INCHES) / Math.tan(totalAngleRad);

                        telemetry.addLine("AprilTag Detected");
                        telemetry.addData("ID", tag.getFiducialId());
                        telemetry.addData("Vertical Angle (ty)", "%.2f°", ty);
                        telemetry.addData("Distance to Tag", "%.2f in", distance);
                        break;
                    }
                }
            } else {
                telemetry.addLine("No valid AprilTag detected.");
            }

            telemetry.update();
        }

        limelight.stop();
    }
}
