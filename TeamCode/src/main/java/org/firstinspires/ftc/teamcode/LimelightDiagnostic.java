package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.mechanisms.LimelightUtil;

/**
 * Limelight3A Diagnostic using official pattern
 *
 * Tests connection and displays detection data
 */
@TeleOp(name = "Limelight3A Diagnostic")
public class LimelightDiagnostic extends OpMode {

    private LimelightUtil limelight;
    private int frameCount = 0;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing Limelight3A...");
        telemetry.update();

        limelight = new LimelightUtil(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Connection", limelight.isConnected() ? "✓ Connected" : "✗ Not Connected");
        telemetry.update();

        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    @Override
    public void loop() {
        frameCount++;

        // IMPORTANT: Update Limelight data
        limelight.update();

        telemetry.addData("=== LIMELIGHT3A DIAGNOSTIC ===", "");
        telemetry.addData("Frame", frameCount);
        telemetry.addData("Connected", limelight.isConnected() ? "YES" : "NO");
        telemetry.addData("", "");

        // Simple status line
        telemetry.addData("Status", limelight.getStatusString());
        telemetry.addData("", "");

        if (limelight.hasTarget()) {
            telemetry.addData("=== TARGET DATA ===", "");
            telemetry.addData("Tag ID", limelight.getTrackedTagId());
            telemetry.addData("", "");

            telemetry.addData("Distance (m)", String.format("%.3f", limelight.getDistanceToTargetMeters()));
            telemetry.addData("Distance (in)", String.format("%.1f", limelight.getDistanceToTargetInches()));
            telemetry.addData("Height (m)", String.format("%.3f", limelight.getHeightToTargetMeters()));
            telemetry.addData("Height (in)", String.format("%.1f", limelight.getHeightToTargetInches()));
            telemetry.addData("", "");

            telemetry.addData("Angle X (tx)", String.format("%.2f°", limelight.getTx()));
            telemetry.addData("Angle Y (ty)", String.format("%.2f°", limelight.getTy()));
            telemetry.addData("", "");

            telemetry.addData("Horizontally Aligned",
                    limelight.isHorizontallyAligned(3.0) ? "YES" : "NO");
            telemetry.addData("Vertically Aligned",
                    limelight.isVerticallyAligned(3.0) ? "YES" : "NO");
        } else {
            telemetry.addData("", "");
            telemetry.addData("Waiting for AprilTag...", "");
            telemetry.addData("", "");
            telemetry.addData("Checklist:", "");
            telemetry.addData("  [ ] Limelight powered on?", "");
            telemetry.addData("  [ ] AprilTag in camera view?", "");
            telemetry.addData("  [ ] Tag 12-48 inches away?", "");
            telemetry.addData("  [ ] Standard FTC AprilTag?", "");
            telemetry.addData("  [ ] Hardware config has 'limelight'?", "");
        }

        telemetry.addData("", "");
        telemetry.addData("Controls:", "");
        telemetry.addData("  A: Info", "");

        if (gamepad1.a) {
            telemetry.addData("", "");
            telemetry.addData("UPDATE PATTERN:", "");
            telemetry.addData("  - limelight.update() called each loop", "");
            telemetry.addData("  - Data cached after update()", "");
            telemetry.addData("  - Getters return cached values", "");
            telemetry.addData("", "");
            telemetry.addData("COORDINATES:", "");
            telemetry.addData("  X,Z = horizontal plane", "");
            telemetry.addData("  Y = vertical (height)", "");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}