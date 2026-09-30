package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.mechanisms.DCMotorsTest;
import org.firstinspires.ftc.teamcode.mechanisms.LimelightUtil;
import org.firstinspires.ftc.teamcode.mechanisms.IMUUtil;

/**
 * LimelightPractice using official Limelight3A pattern
 *
 * KEY: Must call limelight.update() in loop before using getters
 *
 * Controls:
 * - Left Stick Y: Forward/Backward
 * - Right Stick X: Manual turning
 * - X Button: Toggle auto-align
 * - A Button: Toggle auto-drive to target distance
 * - B Button: Stop auto modes
 */
@TeleOp
public class LimelightPractice extends OpMode {

    private DCMotorsTest driveMotors;
    private LimelightUtil limelight;
    private IMUUtil imu;

    private boolean autoAlignMode = false;
    private boolean autoDriveMode = false;
    private double targetDistanceMeters = 1.016;  // 0.75 meters = ~30 inches

    private static final double DEADZONE = 0.1;
    private static final double ALIGNMENT_TOLERANCE = 4.0;  // degrees

    // Auto-align tuning (values are before DCMotorsTest divides power by 4)
    private static final double TURN_KP = -0.02;         // power per degree of tx
    private static final double TURN_KD = -0.003;       // damping per deg/sec of tx change (same sign as TURN_KP); raise if it still swings
    private static final double TURN_MIN_POWER = 0.25;  // enough to overcome friction; raise if it stops short
    private static final double TURN_MAX_POWER = 0.5;   // lower if it overshoots
    private static final double CAMERA_TX_OFFSET = 2.35; // tx reading when robot is truly centered (camera mounted off-center)
    private final ElapsedTime turnTimer = new ElapsedTime();
    private double lastTxError = Double.NaN;  // NaN = no previous reading, skip damping
    private boolean xButtonPressed = false;
    private boolean aButtonPressed = false;

    @Override
    public void init() {
        // Initialize drive motors
        driveMotors = new DCMotorsTest();
        driveMotors.init(hardwareMap);

        // Initialize Limelight with official pattern
        limelight = new LimelightUtil(hardwareMap);
        limelight.setTargetTagId(-1);  // Track any visible tag

        // Initialize IMU
        imu = new IMUUtil();
        imu.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Limelight", limelight.isConnected() ? "Connected" : "Not Connected");
        telemetry.update();
    }

    @Override
    public void loop() {
        // IMPORTANT: Update Limelight data FIRST in loop
        limelight.update();

        // Get joystick input
        double moveForward = applyDeadzone(gamepad1.left_stick_y, DEADZONE);
        double manualTurn = applyDeadzone(gamepad1.right_stick_x, DEADZONE);

        // Button controls
        if (gamepad1.x && !xButtonPressed) {
            autoAlignMode = !autoAlignMode;
            xButtonPressed = true;
        } else if (!gamepad1.x) {
            xButtonPressed = false;
        }

        if (gamepad1.a && !aButtonPressed) {
            autoDriveMode = !autoDriveMode;
            aButtonPressed = true;
        } else if (!gamepad1.a) {
            aButtonPressed = false;
        }

        if (gamepad1.b) {
            autoAlignMode = false;
            autoDriveMode = false;
        }

        // Calculate motor powers
        double leftMotorPower = 0;
        double rightMotorPower = 0;

        if (!autoAlignMode || !limelight.hasTarget()) {
            lastTxError = Double.NaN;  // don't compute damping from a stale reading
        }

        if (limelight.hasTarget()) {
            if (autoAlignMode) {
                // Auto-align: turn to face AprilTag
                double turnCorrection = calculateTurnCorrection(limelight.getTx() - CAMERA_TX_OFFSET);

                if (autoDriveMode) {
                    // Auto-drive: maintain distance to tag
                    double distanceError = targetDistanceMeters - limelight.getDistanceInMeters();
                    double driveCorrection = distanceError * 2.0;  // Tune this (2 motors per meter)
                    driveCorrection = Range.clip(driveCorrection, -0.5, 0.5);

                    // Only the drive term is negated; the turn must match align-only mode
                    leftMotorPower = -driveCorrection + turnCorrection;
                    rightMotorPower = -driveCorrection - turnCorrection;
                } else {
                    // Align only, manual forward/back
                    leftMotorPower = moveForward + turnCorrection;
                    rightMotorPower = moveForward - turnCorrection;
                }
            } else if (autoDriveMode) {
                // Auto-drive without alignment
                double distanceError = targetDistanceMeters - limelight.getDistanceToTargetMeters();
                double driveCorrection = distanceError * 2.0;
                driveCorrection = Range.clip(driveCorrection, -0.5, 0.5);

                leftMotorPower = driveCorrection + manualTurn;
                rightMotorPower = driveCorrection - manualTurn;
            } else {
                // Normal manual arcade drive
                leftMotorPower = moveForward + manualTurn;
                rightMotorPower = moveForward - manualTurn;
            }
        } else {
            // No target - normal manual drive
            leftMotorPower = moveForward + manualTurn;
            rightMotorPower = moveForward - manualTurn;
        }

        // Clamp to valid range
        leftMotorPower = Range.clip(leftMotorPower, -1.0, 1.0);
        rightMotorPower = Range.clip(rightMotorPower, -1.0, 1.0);

        driveMotors.setMotorSpeeds(leftMotorPower, rightMotorPower);

        displayTelemetry();
    }

    /**
     * Proportional turn with a minimum power so the robot doesn't stall a few
     * degrees off target, and a tolerance band so it stops instead of jittering.
     */
    private double calculateTurnCorrection(double txError) {
        // Damping: push against how fast tx is changing so the robot slows before center
        double dt = turnTimer.seconds();
        turnTimer.reset();
        double txRate = (Double.isNaN(lastTxError) || dt <= 0) ? 0.0 : (txError - lastTxError) / dt;
        lastTxError = txError;

        if (Math.abs(txError) <= ALIGNMENT_TOLERANCE) {
            return 0.0;
        }
        double proportional = txError * TURN_KP;
        double turn = proportional + txRate * TURN_KD;
        // Only boost to min power when still heading toward the tag, not while damping is braking
        if (Math.abs(turn) < TURN_MIN_POWER && Math.signum(turn) == Math.signum(proportional)) {
            turn = Math.copySign(TURN_MIN_POWER, turn);
        }
        return Range.clip(turn, -TURN_MAX_POWER, TURN_MAX_POWER);
    }

    private double applyDeadzone(double value, double deadzone) {
        if (Math.abs(value) < deadzone) {
            return 0.0;
        }
        return value;
    }

    private void displayTelemetry() {
        // Status line (one simple line for limelight)
        telemetry.addData("Limelight", limelight.getStatusString());

        telemetry.addData("", "");

        if (limelight.hasTarget()) {
            telemetry.addData("Target ID", limelight.getTrackedTagId());
            telemetry.addData("Distance", String.format("%.2f m (%.1f in)",
                    limelight.getDistanceToTargetMeters(),
                    limelight.getDistanceToTargetInches()));
            telemetry.addData("Height", String.format("%.2f m", limelight.getHeightToTargetMeters()));
            telemetry.addData("Aligned", limelight.isHorizontallyAligned(ALIGNMENT_TOLERANCE) ? "YES" : "NO");
            telemetry.addData("new distance", limelight.getDistance());
            telemetry.addData("Target X", limelight.getTx());
            telemetry.addData("Target Area", limelight.getTargetArea());
            telemetry.addData("new distance **", limelight.getDistanceFromTag());
        }

        telemetry.addData("", "");
        telemetry.addData("Auto-Align", autoAlignMode ? "ON" : "OFF");
        telemetry.addData("Auto-Drive", autoDriveMode ? "ON" : "OFF");

        if (autoDriveMode) {
            telemetry.addData("Target Distance", String.format("%.2f m", targetDistanceMeters));
        }

        telemetry.addData("", "");
        telemetry.addData("X: Align | A: Drive | B: Stop", "");

        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}