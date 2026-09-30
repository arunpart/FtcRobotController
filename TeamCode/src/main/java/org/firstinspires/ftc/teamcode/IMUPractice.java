package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.mechanisms.DCMotorsTest;
import org.firstinspires.ftc.teamcode.mechanisms.IMUUtil;

/**
 * Example TeleOp that demonstrates IMU usage with arcade drive control.
 * Designed for REV Control Hub 2 with built-in IMU.
 *
 * Controls:
 * - Left Stick Y: Forward/Backward movement
 * - Right Stick X: Manual turning
 * - A Button: Reset heading to 0 degrees
 * - B Button: Toggle heading lock mode (maintains current heading while moving)
 */
@TeleOp
public class IMUPractice extends OpMode {

    private DCMotorsTest driveMotors;
    private IMUUtil imu;
    private boolean headingLockMode = false;
    private double targetHeading = 0;
    private static final double DEADZONE = 0.1;
    private static final double HEADING_TOLERANCE = 3.0;  // degrees
    private boolean bButtonPressed = false;
    private boolean yButtonPressed = false;

    @Override
    public void init() {
        driveMotors = new DCMotorsTest();
        driveMotors.init(hardwareMap);

        imu = new IMUUtil();
        // Initialize with default orientation (Logo UP, USB FORWARD)
        // Change these parameters if your Control Hub is mounted differently
        imu.init(hardwareMap,
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

        telemetry.addData("Status", "Robot Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Get joystick input
        double moveForward = applyDeadzone(gamepad1.left_stick_y, DEADZONE);
        double manualTurn = applyDeadzone(gamepad1.right_stick_x, DEADZONE);

        // Button controls with debouncing
        if (gamepad1.a) {
            // Reset heading to 0
            imu.resetHeading();
            targetHeading = 0;
        }

        if (gamepad1.b && !bButtonPressed) {
            // Toggle heading lock mode (debounced)
            headingLockMode = !headingLockMode;
            if (headingLockMode) {
                targetHeading = imu.getHeading();
            }
            bButtonPressed = true;
        } else if (!gamepad1.b) {
            bButtonPressed = false;
        }

        if (gamepad1.y && !yButtonPressed) {
            // Y button: turn robot 180 degrees from current heading (debounced)
            double currentHeading = imu.getHeading();
            targetHeading = currentHeading + 180;

            // Normalize to -180 to 180 range
            while (targetHeading > 180) {
                targetHeading -= 360;
            }
            while (targetHeading < -180) {
                targetHeading += 360;
            }

            headingLockMode = true;
            yButtonPressed = true;
        } else if (!gamepad1.y) {
            yButtonPressed = false;
        }

        // Calculate motor powers
        double leftMotorPower, rightMotorPower;

        if (headingLockMode) {
            // Heading lock mode: maintain target heading while moving
            double currentHeading = imu.getHeading();
            double headingError = imu.getHeadingDifference(targetHeading, currentHeading);

            // Proportional control for heading correction
            // Tune this multiplier (0.01) based on how aggressive you want the correction
            double headingCorrection = headingError * 0.01;
            headingCorrection = Range.clip(headingCorrection, -0.5, 0.5);

            // Combine forward movement with heading correction
            leftMotorPower = moveForward + headingCorrection;
            rightMotorPower = moveForward - headingCorrection;
        } else {
            // Normal arcade drive with manual turning
            leftMotorPower = moveForward + manualTurn;
            rightMotorPower = moveForward - manualTurn;
        }

        // Clamp motor powers
        leftMotorPower = Range.clip(leftMotorPower, -1.0, 1.0);
        rightMotorPower = Range.clip(rightMotorPower, -1.0, 1.0);

        // Set motor speeds
        driveMotors.setMotorSpeeds(leftMotorPower, rightMotorPower);

        // Telemetry
        displayTelemetry(moveForward, manualTurn, leftMotorPower, rightMotorPower);
    }

    /**
     * Apply deadzone to joystick input
     */
    private double applyDeadzone(double value, double deadzone) {
        if (Math.abs(value) < deadzone) {
            return 0.0;
        }
        return value;
    }

    /**
     * Display comprehensive telemetry data
     */
    private void displayTelemetry(double forward, double turn, double leftPower, double rightPower) {
        double currentHeading = imu.getHeading();
        double pitch = imu.getPitch();
        double roll = imu.getRoll();

        telemetry.addData("=== DRIVE CONTROLS ===", "");
        telemetry.addData("Forward Input", String.format("%.3f", forward));
        telemetry.addData("Turn Input", String.format("%.3f", turn));
        telemetry.addData("Left Motor Power", String.format("%.3f", leftPower));
        telemetry.addData("Right Motor Power", String.format("%.3f", rightPower));

        telemetry.addData("=== IMU ORIENTATION ===", "");
        telemetry.addData("Heading (Yaw)", String.format("%.1f°", currentHeading));
        telemetry.addData("Pitch", String.format("%.1f°", pitch));
        telemetry.addData("Roll", String.format("%.1f°", roll));

        telemetry.addData("=== HEADING LOCK MODE ===", "");
        telemetry.addData("Mode Active", headingLockMode ? "ON" : "OFF");
        if (headingLockMode) {
            telemetry.addData("Target Heading", String.format("%.1f°", targetHeading));
            telemetry.addData("Heading Error", String.format("%.1f°", imu.getHeadingDifference(targetHeading, currentHeading)));
            telemetry.addData("At Target", imu.isAtHeading(targetHeading, HEADING_TOLERANCE));
        }

        telemetry.addData("=== ROBOT STATUS ===", "");
        telemetry.addData("Robot Level", imu.isLevel(5.0) ? "YES" : "NO");

        telemetry.addData("=== BUTTON CONTROLS ===", "");
        telemetry.addData("A: Reset Heading | B: Toggle Lock", "");
        telemetry.addData("Y: Turn 180°", "");

        telemetry.update();
    }
}