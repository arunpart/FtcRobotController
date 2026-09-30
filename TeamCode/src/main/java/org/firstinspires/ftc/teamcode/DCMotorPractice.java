package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.mechanisms.DCMotorsTest;

@TeleOp
public class DCMotorPractice extends OpMode {

    DCMotorsTest aMotor = new DCMotorsTest();

    // Deadzone constant - adjust to reduce stick drift
    private static final double DEADZONE = 0.1;

    @Override
    public void init() {
        aMotor.init(hardwareMap);
        telemetry.addData("Status", "Robot Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Get gamepad input
        double moveForward = gamepad1.left_stick_y;   // Forward/backward movement
        double turn = gamepad1.right_stick_x;         // Turning/rotation

        // Apply deadzone to avoid small drift
        moveForward = applyDeadzone(moveForward, DEADZONE);
        turn = applyDeadzone(turn, DEADZONE);

        // Arcade drive calculation
        // For tank drive: forward + turn = left motor, forward - turn = right motor
        double leftPower = moveForward + turn;
        double rightPower = moveForward - turn;

        // Clamp values to [-1, 1] range
        leftPower = Range.clip(leftPower, -1.0, 1.0);
        rightPower = Range.clip(rightPower, -1.0, 1.0);

        // Set motor speeds
        aMotor.setMotorSpeeds(leftPower, rightPower);

        // Telemetry for debugging
        telemetry.addData("Left Stick Y (Forward)", moveForward);
        telemetry.addData("Right Stick X (Turn)", turn);
        telemetry.addData("Left Motor Power", leftPower);
        telemetry.addData("Right Motor Power", rightPower);
        telemetry.addData("Left Rev", aMotor.getLeftMotorsRev());
        telemetry.addData("Right Rev", aMotor.getRightMotorsRev());
        telemetry.addData("L Ticks", aMotor.getLeftTics());
        telemetry.addData("R Ticks", aMotor.getRightTics());
        telemetry.update();
    }

    /**
     * Apply deadzone to joystick input to avoid drift
     * @param value The raw joystick value
     * @param deadzone The deadzone threshold (typically 0.05-0.15)
     * @return The adjusted value with deadzone applied
     */
    private double applyDeadzone(double value, double deadzone) {
        if (Math.abs(value) < deadzone) {
            return 0.0;
        }
        return value;
    }
}
