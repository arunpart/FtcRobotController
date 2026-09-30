package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * Utility class for reading and managing IMU (Inertial Measurement Unit) data from REV Control Hub 2.
 * Provides robot orientation information including heading, pitch, and roll angles.
 *
 * The IMU tracks the robot's rotation relative to its starting orientation.
 * - Heading (Yaw): Rotation around vertical axis (0-360 degrees)
 * - Pitch: Rotation forward/backward around horizontal axis
 * - Roll: Rotation left/right around forward axis
 */
public class IMUUtil {

    private IMU imu;
    private YawPitchRollAngles lastAngles;

    // Constants
    private static final String IMU_NAME = "imu";

    /**
     * Initialize the IMU with specified orientation.
     * Configure based on how your Control Hub is mounted on the robot.
     *
     * @param hwMap HardwareMap from OpMode
     * @param logoDirection Direction the REV logo is facing
     * @param usbDirection Direction the USB ports are facing
     */
    public void init(HardwareMap hwMap,
                     RevHubOrientationOnRobot.LogoFacingDirection logoDirection,
                     RevHubOrientationOnRobot.UsbFacingDirection usbDirection) {

        // Get the IMU from hardware map
        imu = hwMap.get(IMU.class, IMU_NAME);

        // Define how the Rev Hub is mounted on the robot
        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Initialize IMU with orientation
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Update last known angles
        lastAngles = imu.getRobotYawPitchRollAngles();
    }

    /**
     * Initialize the IMU with default orientation (Logo UP, USB FORWARD).
     * Use this if your Control Hub is mounted normally (logo facing up, USB facing forward).
     *
     * @param hwMap HardwareMap from OpMode
     */
    public void init(HardwareMap hwMap) {
        this.init(hwMap,
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
    }

    /**
     * Get the robot's heading (yaw angle) in degrees.
     * Range: -180 to 180 degrees
     * 0° = initial orientation
     * Positive = counterclockwise rotation (when viewed from above)
     *
     * @return Heading in degrees
     */
    public double getHeading() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        lastAngles = angles;
        return angles.getYaw(AngleUnit.DEGREES);
    }

    /**
     * Get the robot's heading normalized to 0-360 range.
     *
     * @return Heading in degrees (0-360)
     */
    public double getHeading0to360() {
        double heading = getHeading();
        if (heading < 0) {
            heading += 360;
        }
        return heading;
    }

    /**
     * Get the robot's pitch angle in degrees.
     * Positive = tilted forward
     * Negative = tilted backward
     *
     * @return Pitch in degrees
     */
    public double getPitch() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        lastAngles = angles;
        return angles.getPitch(AngleUnit.DEGREES);
    }

    /**
     * Get the robot's roll angle in degrees.
     * Positive = tilted right
     * Negative = tilted left
     *
     * @return Roll in degrees
     */
    public double getRoll() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        lastAngles = angles;
        return angles.getRoll(AngleUnit.DEGREES);
    }

    /**
     * Get all orientation angles at once (more efficient than calling separately).
     *
     * @return YawPitchRollAngles object containing heading, pitch, and roll
     */
    public YawPitchRollAngles getAllAngles() {
        lastAngles = imu.getRobotYawPitchRollAngles();
        return lastAngles;
    }

    /**
     * Calculate the angle difference between two headings, taking into account wrap-around.
     * Returns the shortest rotation path.
     *
     * @param targetHeading Target heading in degrees
     * @param currentHeading Current heading in degrees
     * @return Angle difference in degrees (-180 to 180)
     */
    public double getHeadingDifference(double targetHeading, double currentHeading) {
        double difference = targetHeading - currentHeading;

        // Normalize to -180 to 180 range
        while (difference > 180) {
            difference -= 360;
        }
        while (difference < -180) {
            difference += 360;
        }

        return difference;
    }

    /**
     * Check if robot is at target heading (within tolerance).
     *
     * @param targetHeading Target heading in degrees
     * @param tolerance Tolerance in degrees (default 2-5 degrees is reasonable)
     * @return true if within tolerance of target heading
     */
    public boolean isAtHeading(double targetHeading, double tolerance) {
        double difference = Math.abs(getHeadingDifference(targetHeading, getHeading()));
        return difference <= tolerance;
    }

    /**
     * Check if robot is level (pitch and roll are near zero).
     *
     * @param tolerance Tolerance in degrees
     * @return true if robot is level within tolerance
     */
    public boolean isLevel(double tolerance) {
        YawPitchRollAngles angles = getAllAngles();
        return Math.abs(angles.getPitch(AngleUnit.DEGREES)) <= tolerance &&
                Math.abs(angles.getRoll(AngleUnit.DEGREES)) <= tolerance;
    }

    /**
     * Reset the IMU heading to 0 degrees.
     * Call this when you want to recalibrate the starting heading.
     */
    public void resetHeading() {
        imu.resetYaw();
    }

    /**
     * Get last known angles (without querying IMU).
     * Useful to avoid frequent IMU queries in tight loops.
     *
     * @return Last YawPitchRollAngles retrieved
     */
    public YawPitchRollAngles getLastAngles() {
        return lastAngles;
    }

    /**
     * Get the underlying IMU object for direct access if needed.
     *
     * @return IMU object
     */
    public IMU getIMU() {
        return imu;
    }
}