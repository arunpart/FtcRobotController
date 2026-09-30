package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.mechanisms.DCMotorsTest;
import org.firstinspires.ftc.teamcode.mechanisms.IMUUtil;
import org.firstinspires.ftc.teamcode.mechanisms.LimelightUtil;

/**
 * Autonomous: drive straight forward for 2 seconds, turn right 90 degrees with the IMU,
 * then auto-align to an AprilTag with the Limelight.
 *
 * Sign conventions match the other OpModes in this project:
 * - Negative power = forward (same as pushing left_stick_y forward)
 * - left = forward + turn, right = forward - turn
 */
@Autonomous
public class DriveStraightTimedAuto extends LinearOpMode {

    private DCMotorsTest driveMotors;
    private IMUUtil imu;
    private LimelightUtil limelight;

    // Values are before DCMotorsTest divides power by 4
    private static final double DRIVE_SECONDS = 2.2;
    private static final double DRIVE_POWER = 0.2;
    private static final double DRIVE_HEADING_KP = 0.01;  // keeps the robot straight; flip sign if it veers more

    // Turn settings
    private static final double TURN_DEGREES = -90.0;      // IMU heading: positive = left, negative = right
    // Negative on this robot: a positive turn value spins it LEFT (same reason LimelightPractice uses a negative TURN_KP)
    private static final double TURN_KP = -0.03;           // power per degree of heading error
    private static final double TURN_MIN_POWER = 0.25;     // enough to overcome friction; raise if it stops short
    private static final double TURN_MAX_POWER = 0.6;      // lower if it overshoots
    private static final double HEADING_TOLERANCE = 1.5;   // degrees
    private static final double SETTLE_SECONDS = 0.25;     // must stay in tolerance this long to finish
    private static final double TURN_TIMEOUT_SECONDS = 1.0;
    private static final long PAUSE_BETWEEN_STEPS_MS = 300;  // full stop between driving and turning

    // Auto-align settings (copied from LimelightPractice, where they were tuned on the robot)
    private static final double ALIGN_TOLERANCE = 4.0;      // degrees of tx
    private static final double ALIGN_KP = -0.02;           // power per degree of tx
    private static final double ALIGN_KD = -0.003;          // damping per deg/sec of tx change (same sign as ALIGN_KP)
    private static final double ALIGN_MIN_POWER = 0.25;
    private static final double ALIGN_MAX_POWER = 0.5;
    private static final double CAMERA_TX_OFFSET = 2.35;    // tx reading when robot is truly centered
    private static final double ALIGN_TIMEOUT_SECONDS = 3.0;
    private final ElapsedTime alignTimer = new ElapsedTime();
    private double lastTxError = Double.NaN;  // NaN = no previous reading, skip damping


    @Override
    public void runOpMode() {
        driveMotors = new DCMotorsTest();
        driveMotors.init(hardwareMap);

        imu = new IMUUtil();
        imu.init(hardwareMap);

        limelight = new LimelightUtil(hardwareMap);
        limelight.setTargetTagId(-1);  // align to any visible tag

        telemetry.addData("Status", "Initialized - press START");
        telemetry.addData("Limelight", limelight.isConnected() ? "Connected" : "Not Connected");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        imu.resetHeading();
        ElapsedTime timer = new ElapsedTime();
        telemetry.addData("timer", timer.seconds());

        while (opModeIsActive() && timer.seconds() < DRIVE_SECONDS) {
            double headingError = imu.getHeadingDifference(0, imu.getHeading());
            //double correction = Range.clip(-headingError * DRIVE_HEADING_KP, -0.2, 0.2);

            double forward = -DRIVE_POWER;  // negative = forward
            driveMotors.setMotorSpeeds(forward , forward);

            telemetry.addData("Step", "Driving");
            telemetry.addData("Time", String.format("%.1f / %.1f s", timer.seconds(), DRIVE_SECONDS));
            telemetry.addData("Heading Error", String.format("%.1f°", headingError));
            telemetry.update();
        }

        driveMotors.stopMotors();

        sleep(PAUSE_BETWEEN_STEPS_MS);
        String turnResult = turnToHeading(TURN_DEGREES);
        driveMotors.stopMotors();

        sleep(PAUSE_BETWEEN_STEPS_MS);
        String alignResult = autoAlign();
        driveMotors.stopMotors();


        while (opModeIsActive()) {
            telemetry.addData("Status", "Done");
            telemetry.addData("Turn", turnResult);
            telemetry.addData("Auto-Align", alignResult);
            telemetry.addData("Final Heading", String.format("%.1f°", imu.getHeading()));
            telemetry.update();
            sleep(50);
        }
        limelight.stop();
    }

    /**
     * Turn in place until the AprilTag is centered in the Limelight view.
     * Uses the same control as LimelightPractice auto-align.
     *
     * @return a short description of how the alignment ended, for telemetry
     */
    private String autoAlign() {
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime settleTimer = new ElapsedTime();
        lastTxError = Double.NaN;
        alignTimer.reset();

        while (opModeIsActive()) {
            if (timer.seconds() >= ALIGN_TIMEOUT_SECONDS) {
                driveMotors.stopMotors();
                return limelight.hasTarget()
                        ? String.format("TIMED OUT at tx %.1f°", limelight.getTx() - CAMERA_TX_OFFSET)
                        : "NO TAG SEEN - check the tag is in view after the turn";
            }

            // Must be called every loop before reading Limelight data
            limelight.update();

            double turn = 0.0;
            if (!limelight.hasTarget()) {
                // Don't guess which way to turn - wait for the tag
                lastTxError = Double.NaN;
                settleTimer.reset();
            } else {
                double txError = limelight.getTx() - CAMERA_TX_OFFSET;
                turn = calculateAlignCorrection(txError);

                if (Math.abs(txError) <= ALIGN_TOLERANCE) {
                    if (settleTimer.seconds() >= SETTLE_SECONDS) {
                        driveMotors.stopMotors();
                        return String.format("Aligned to tag %d (tx %.1f°)", limelight.getTrackedTagId(), txError);
                    }
                } else {
                    settleTimer.reset();
                }
            }

            driveMotors.setMotorSpeeds(turn, -turn);

            telemetry.addData("Step", "Auto-Align");
            telemetry.addData("Tag", limelight.hasTarget() ? limelight.getTrackedTagId() : "none");
            telemetry.addData("tx", String.format("%.1f°", limelight.getTx() - CAMERA_TX_OFFSET));
            telemetry.addData("Turn Power", String.format("%.2f", turn));
            telemetry.update();
        }
        driveMotors.stopMotors();
        return "Stopped";
    }

    /**
     * Same as LimelightPractice.calculateTurnCorrection: proportional turn with damping,
     * a minimum power so it doesn't stall, and a tolerance band so it stops cleanly.
     */
    private double calculateAlignCorrection(double txError) {
        // Damping: push against how fast tx is changing so the robot slows before center
        double dt = alignTimer.seconds();
        alignTimer.reset();
        double txRate = (Double.isNaN(lastTxError) || dt <= 0) ? 0.0 : (txError - lastTxError) / dt;
        lastTxError = txError;

        if (Math.abs(txError) <= ALIGN_TOLERANCE) {
            return 0.0;
        }
        double proportional = txError * ALIGN_KP;
        double turn = proportional + txRate * ALIGN_KD;
        // Only boost to min power when still heading toward the tag, not while damping is braking
        if (Math.abs(turn) < ALIGN_MIN_POWER && Math.signum(turn) == Math.signum(proportional)) {
            turn = Math.copySign(ALIGN_MIN_POWER, turn);
        }
        return Range.clip(turn, -ALIGN_MAX_POWER, ALIGN_MAX_POWER);
    }


    /**
     * Turn in place to the given heading (relative to where the robot started).
     * Positive = left, negative = right.
     *
     * @return a short description of how the turn ended, for telemetry
     */
    private String turnToHeading(double targetHeading) {
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime settleTimer = new ElapsedTime();
        double startHeading = imu.getHeading();
        double startError = Math.abs(imu.getHeadingDifference(targetHeading, startHeading));

        // Keep correcting until the heading is on target (or we time out)
        while (opModeIsActive()) {
            if (timer.seconds() >= TURN_TIMEOUT_SECONDS) {
                driveMotors.stopMotors();
                return String.format("TIMED OUT at %.1f°", imu.getHeading());
            }

            double heading = imu.getHeading();
            // Target stays fixed; only the error changes as the robot turns
            double error = imu.getHeadingDifference(targetHeading, heading);  // negative = need to turn right

            // Safety checks: stop instead of spinning in circles
            if (Math.abs(error) > startError + 20) {
                driveMotors.stopMotors();
                return String.format("WRONG WAY (heading %.1f°) - flip the sign of TURN_KP", heading);
            }
            if (timer.seconds() > 0.75 && Math.abs(imu.getHeadingDifference(heading, startHeading)) < 2) {
                driveMotors.stopMotors();
                return "IMU NOT SEEING TURN - check hub orientation in IMUUtil.init, or raise TURN_MIN_POWER";
            }

            double turn = 0.0;
            if (Math.abs(error) <= HEADING_TOLERANCE) {
                if (settleTimer.seconds() >= SETTLE_SECONDS) {
                    driveMotors.stopMotors();
                    return String.format("Reached %.1f°", heading);
                }
            } else {
                settleTimer.reset();
                turn = -error * TURN_KP;  // with TURN_KP negative: error -90 -> turn -0.6 -> spins right on this robot
                if (Math.abs(turn) < TURN_MIN_POWER) {
                    turn = Math.copySign(TURN_MIN_POWER, turn);
                }
                turn = Range.clip(turn, -TURN_MAX_POWER, TURN_MAX_POWER);
            }

            // Opposite powers on each side = spin in place
            driveMotors.setMotorSpeeds(turn, -turn);

            telemetry.addData("Step", "Turning");
            telemetry.addData("Heading", String.format("%.1f° -> %.1f°", heading, targetHeading));
            telemetry.addData("Error", String.format("%.1f°", error));
            telemetry.addData("Turn Power", String.format("%.2f", turn));
            telemetry.update();
        }
        driveMotors.stopMotors();
        return "Stopped";
    }
}
