package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.mechanisms.DCMotorsTest;
import org.firstinspires.ftc.teamcode.mechanisms.IMUUtil;

/**
 * Autonomous: drive forward 3 wheel revolutions, then turn left 15 degrees using the IMU.
 *
 * Sign conventions match the TeleOps in this project:
 * - Negative power = forward (same as pushing left_stick_y forward)
 * - left = forward + turn, right = forward - turn, so negative turn = turn left
 * - IMU heading: positive = counterclockwise (left)
 */
@Autonomous
public class DriveAndTurnAuto extends LinearOpMode {

    private DCMotorsTest driveMotors;
    private IMUUtil imu;

    // Drive settings (values are before DCMotorsTest divides power by 4)
    private static final double DRIVE_REVOLUTIONS = 3.0;
    private static final double DRIVE_POWER = 0.2;
    private static final double DRIVE_MIN_POWER = 0.3;    // slowest speed while ramping down; raise if it stalls before 3 revs
    private static final double SLOWDOWN_REVOLUTIONS = 0.75; // start slowing down this far before the target
    private static final double DRIVE_HEADING_KP = 0.02;  // keeps the robot straight while driving
    private static final double DRIVE_TIMEOUT_SECONDS = 2.0;

    // Turn settings
    private static final double TURN_DEGREES = 15.0;       // positive = left
    private static final double TURN_KP = 0.03;            // power per degree of heading error
    private static final double TURN_MIN_POWER = 0.25;     // enough to overcome friction; raise if it stops short
    private static final double TURN_MAX_POWER = 0.6;      // lower if it overshoots
    private static final double HEADING_TOLERANCE = 1.5;   // degrees
    private static final double SETTLE_SECONDS = 0.25;     // must stay in tolerance this long to finish
    private static final double TURN_TIMEOUT_SECONDS = 1.0;
    private static final long PAUSE_BETWEEN_STEPS_MS = 300;  // full stop between driving and turning

    @Override
    public void runOpMode() {
        driveMotors = new DCMotorsTest();
        driveMotors.init(hardwareMap);

        imu = new IMUUtil();
        imu.init(hardwareMap);

        telemetry.addData("Status", "Initialized - press START");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        imu.resetHeading();

        String driveResult = driveRevolutions(DRIVE_REVOLUTIONS, DRIVE_POWER);

        // Come to a complete stop before turning
        driveMotors.stopMotors();
        //sleep(PAUSE_BETWEEN_STEPS_MS);

        //String turnResult = turnToHeading(TURN_DEGREES);
        //driveMotors.stopMotors();

        // Keep the result on screen until the OpMode is stopped
        while (opModeIsActive()) {
            telemetry.addData("Status", "Done");
            telemetry.addData("Drive", driveResult);
            //telemetry.addData("Turn", turnResult);
            telemetry.addData("Final Heading", String.format("%.1f°", imu.getHeading()));
            telemetry.update();
            sleep(50);
        }
    }

    /**
     * Drive straight forward until the wheels have turned the given number of revolutions,
     * holding the heading that the robot started this step with. Slows down near the
     * target so it stops on the mark instead of coasting past.
     *
     * @return a short description of how the step ended, for telemetry
     */
    private String driveRevolutions(double revolutions, double power) {
        double startLeft = driveMotors.getLeftMotorsRev();
        double startRight = driveMotors.getRightMotorsRev();
        double holdHeading = imu.getHeading();
        ElapsedTime timer = new ElapsedTime();

        double traveled = 0.0;

        while (opModeIsActive()) {
            // abs() so this works whichever way each encoder counts
            double leftDone = Math.abs(driveMotors.getLeftMotorsRev() - startLeft);
            double rightDone = Math.abs(driveMotors.getRightMotorsRev() - startRight);
            traveled = (leftDone + rightDone) / 2.0;
            if (traveled >= revolutions) {
                driveMotors.stopMotors();
                return String.format("Reached %.2f revs", traveled);
            }
            if (timer.seconds() >= DRIVE_TIMEOUT_SECONDS) {
                driveMotors.stopMotors();
                return String.format("TIMED OUT at %.2f revs", traveled);
            }

            // Ramp down linearly over the last SLOWDOWN_REVOLUTIONS, but never below DRIVE_MIN_POWER
            double remaining = revolutions - traveled;
            double speed = power;
            if (remaining < SLOWDOWN_REVOLUTIONS) {
                speed = Math.max(Math.min(power, DRIVE_MIN_POWER), power * remaining / SLOWDOWN_REVOLUTIONS);
            }

            double headingError = imu.getHeadingDifference(holdHeading, imu.getHeading());
            double correction = Range.clip(-headingError * DRIVE_HEADING_KP, -0.2, 0.2);

            double forward = -speed;  // negative = forward
            driveMotors.setMotorSpeeds(forward + correction, forward - correction);

            telemetry.addData("Step", "Driving");
            telemetry.addData("Revolutions", String.format("%.2f / %.2f", traveled, revolutions));
            telemetry.addData("Heading Error", String.format("%.1f°", headingError));
            telemetry.addData("Power", String.format("%.2f", speed));
            telemetry.update();
        }
        driveMotors.stopMotors();
        return String.format("Stopped at %.2f revs", traveled);
    }

    /**
     * Turn in place to the given heading (relative to the start of the match).
     * Positive = left, negative = right.
     *
     * @return a short description of how the step ended, for telemetry
     */
    private String turnToHeading(double targetHeading) {
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime settleTimer = new ElapsedTime();

        while (opModeIsActive()) {
            if (timer.seconds() >= TURN_TIMEOUT_SECONDS) {
                driveMotors.stopMotors();
                return String.format("TIMED OUT at %.1f°", imu.getHeading());
            }
            double heading = imu.getHeading();
            double error = imu.getHeadingDifference(targetHeading, heading);  // positive = need to turn left

            double turn = 0.0;
            if (Math.abs(error) <= HEADING_TOLERANCE) {
                if (settleTimer.seconds() >= SETTLE_SECONDS) {
                    driveMotors.stopMotors();
                    return String.format("Reached %.1f°", heading);
                }
            } else {
                settleTimer.reset();
                turn = -error * TURN_KP;  // negative turn = left
                if (Math.abs(turn) < TURN_MIN_POWER) {
                    turn = Math.copySign(TURN_MIN_POWER, turn);
                }
                turn = Range.clip(turn, -TURN_MAX_POWER, TURN_MAX_POWER);
            }

            driveMotors.setMotorSpeeds(turn, -turn);

            telemetry.addData("Step", "Turning");
            telemetry.addData("Heading", String.format("%.1f° -> %.1f°", heading, targetHeading));
            telemetry.addData("Error", String.format("%.1f°", error));
            telemetry.update();
        }
        driveMotors.stopMotors();
        return "Stopped";
    }
}
