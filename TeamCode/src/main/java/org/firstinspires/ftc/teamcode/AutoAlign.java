package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

@TeleOp(name="AprilTag Align TeleOp", group="TeleOp")
public class AutoAlign extends LinearOpMode {

    // Robot and vision constants
    private static final double TAG_HEIGHT = 60.0;    // inches (AprilTag height)
    private static final double CAM_HEIGHT = 20.0;    // inches (camera height)
    private static final double CAM_ANGLE  = 25.0;    // degrees (camera tilt angle)
    private static final int TARGET_ID = 20;          // AprilTag ID to align to

    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        // Initialize drive motors
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        // Configure motor directions (adjust as needed for robot configuration)
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // Use encoders for more consistent control
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Brake mode for precise stopping
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize Limelight 3A
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11); // fastest telemetry rate:contentReference[oaicite:0]{index=0}

        // Set Limelight to AprilTag pipeline (index 4) and fast polling
        limelight.pipelineSwitch(4);  // switch to pipeline 4 (AprilTags):contentReference[oaicite:1]{index=1}
        limelight.setPollRateHz(100); // set high polling rate (100Hz):contentReference[oaicite:2]{index=2}

        waitForStart();
        limelight.start(); // begin polling Limelight data

        boolean aligning = false;        // are we aligning to the tag?
        boolean xButtonPrev = false;     // previous state of gamepad X button

        while (opModeIsActive()) {
            // Toggle auto-align mode when X is pressed
            if (gamepad1.x && !xButtonPrev) {
                aligning = true;
            }
            xButtonPrev = gamepad1.x;

            // Retrieve latest Limelight result
            LLResult result = limelight.getLatestResult();
            double tx = 0, ty = 0;
            boolean targetVisible = false;

            if (result != null && result.isValid()) {
                // Check all AprilTag detections for target ID
                List<FiducialResult> fiducials = result.getFiducialResults(); // list of AprilTag results:contentReference[oaicite:3]{index=3}
                for (FiducialResult fid : fiducials) {
                    if (fid.getFiducialId() == TARGET_ID) { // found our tag ID:contentReference[oaicite:4]{index=4}
                        tx = fid.getTargetXDegrees(); // horizontal offset to target (deg):contentReference[oaicite:5]{index=5}
                        ty = fid.getTargetYDegrees(); // vertical offset to target (deg)
                        targetVisible = true;
                        break;
                    }
                }
            }

            if (aligning) {
                if (!targetVisible) {
                    // Rotate robot until tag is found
                    double spinPower = 0.3;
                    leftFront.setPower(spinPower);
                    leftBack.setPower(spinPower);
                    rightFront.setPower(-spinPower);
                    rightBack.setPower(-spinPower);
                } else {
                    // Align to tag using proportional control on tx
                    double kP = 0.02;
                    double turn = Range.clip(kP * tx, -0.4, 0.4);
                    leftFront.setPower(turn);
                    leftBack.setPower(turn);
                    rightFront.setPower(-turn);
                    rightBack.setPower(-turn);

                    // Stop when roughly centered on the tag
                    if (Math.abs(tx) < 1.0) {
                        leftFront.setPower(0);
                        leftBack.setPower(0);
                        rightFront.setPower(0);
                        rightBack.setPower(0);
                        aligning = false;
                    }
                }
            } else {
                // Standard mecanum drive control (no alignment)
                double drive  = -gamepad1.left_stick_y; // forward/back
                double strafe =  gamepad1.left_stick_x; // left/right
                double rotate =  gamepad1.right_stick_x; // rotation
                double fl = drive + strafe + rotate;
                double bl = drive - strafe + rotate;
                double fr = drive - strafe - rotate;
                double br = drive + strafe - rotate;
                // Normalize wheel speeds
                double max = Math.max(1.0, Math.max(Math.abs(fl),
                        Math.max(Math.abs(bl),
                                Math.max(Math.abs(fr), Math.abs(br)))));
                fl /= max;
                bl /= max;
                fr /= max;
                br /= max;
                leftFront.setPower(fl);
                leftBack.setPower(bl);
                rightFront.setPower(fr);
                rightBack.setPower(br);
            }

            // Display distance to tag if visible
            if (targetVisible) {
                double angleToTarget = CAM_ANGLE + ty;
                double angleRad = Math.toRadians(angleToTarget);
                double distanceInches = (TAG_HEIGHT - CAM_HEIGHT) / Math.tan(angleRad); // distance formula:contentReference[oaicite:6]{index=6}
                telemetry.addData("Distance (in)", distanceInches);
            } else {
                telemetry.addData("Distance (in)", "Unknown");
            }

            telemetry.update();
        }
    }
}
