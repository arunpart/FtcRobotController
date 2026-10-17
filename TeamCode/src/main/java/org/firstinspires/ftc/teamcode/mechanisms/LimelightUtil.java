package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;
import java.util.Locale;

/**
 * Utility class for Limelight3A AprilTag detection.
 *
 * IMPORTANT: Call update() in your main loop to cache fresh data.
 * All getter methods return the last cached data.
 *
 * Design pattern based on official Limelight SDK.
 */
public class LimelightUtil {

    private Limelight3A limelight;
    private int targetTagId = -1;  // -1 = track any visible tag

    // Cached data from last update() call
    private double lastDistanceMeters = 0;
    private double lastHeightMeters = 0;
    private double lastTx = 0;
    private double lastTy = 0;
    private double lastTagYaw = 0;
    private int lastTrackedTagId = -1;
    private boolean hasValidTarget = false;

    private double targetArea = 0;

    private String botPose;

    /**
     * Constructor - initialize Limelight3A
     *
     * @param hardwareMap HardwareMap from OpMode
     */
    public LimelightUtil(HardwareMap hardwareMap) {
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100);
            limelight.pipelineSwitch(0);  // Default AprilTag pipeline
            limelight.start();
            System.out.println("Limelight3A initialized successfully");
        } catch (Exception e) {
            System.err.println("Error initializing Limelight3A: " + e.getMessage());
            limelight = null;
        }
    }

    public double getDistance() {
        double cameraAngle = 26.5; // Replace with your camera's mounting angle in degrees
        double cameraHeight = 11.75; // Replace with your camera's height from the floor
        double goalHeight = 53.50;   // Replace with the goal/target height from the floor

        double angleToTarget = cameraAngle + getTy();
        double heightDifference = goalHeight - cameraHeight;

        double distance = heightDifference / Math.tan(Math.toRadians(angleToTarget));
        return distance;
    }

    public double getDistanceInMeters() {

        return getDistance() * 0.0254;
    }

    public double getDistanceFromTag() {
        double scale = 54070.98;
        double distance = (scale/targetArea);
        return distance;

    }


    /**
     * Set which AprilTag to track.
     *
     * @param tagId Specific tag ID to track, or -1 to track any visible tag
     */
    public void setTargetTagId(int tagId) {
        this.targetTagId = tagId;
    }

    /**
     * MUST be called in your main loop to update cached vision data.
     * All getters return data cached from the last update() call.
     */
    public void update() {
        if (limelight == null) {
            hasValidTarget = false;
            return;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            hasValidTarget = false;
            return;
        }

        // Cache raw angle offsets
        lastTx = result.getTx();
        lastTy = result.getTy();
        targetArea = result.getTa();

        // Get list of detected AprilTags (fiducials)
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials.isEmpty()) {
            hasValidTarget = false;
            return;
        }

        // Find target tag (specific ID or first available)
        LLResultTypes.FiducialResult targetFiducial = null;
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            if (targetTagId == -1 || fiducial.getFiducialId() == targetTagId) {
                targetFiducial = fiducial;
                break;
            }
        }

        if (targetFiducial == null) {
            hasValidTarget = false;
            return;
        }

        lastTrackedTagId = targetFiducial.getFiducialId();
        // Use this tag's angles, not the Limelight's primary target
        lastTx = targetFiducial.getTargetXDegrees();
        lastTy = targetFiducial.getTargetYDegrees();

        // Get robot position relative to AprilTag
        Pose3D robotPoseTargetSpace = targetFiducial.getRobotPoseTargetSpace();
        if (robotPoseTargetSpace != null) {
            Position pos = robotPoseTargetSpace.getPosition();

            // X and Z form horizontal plane, Y is vertical
            lastDistanceMeters = Math.sqrt(pos.x * pos.x + pos.z * pos.z);
            lastHeightMeters = pos.y;
            lastTagYaw = robotPoseTargetSpace.getOrientation().getYaw(AngleUnit.DEGREES);

            botPose = robotPoseTargetSpace.toString();

            hasValidTarget = true;
        } else {
            hasValidTarget = false;
        }
    }

    public double getTargetArea(){
        return targetArea;
    }

    public String getBotPose(){
        return botPose;
    }

    /**
     * Get the robot's yaw relative to the AprilTag (from the tag's 3D pose).
     * Unlike tx, this changes when the robot is angled to the tag's surface even if
     * the tag is centered in the image. Check on the robot what it reads when facing
     * the tag squarely (about 0 or about 180) and which way it moves when turning left.
     *
     * @return yaw in degrees, from the last update() with a valid target
     */
    public double getTagYaw() {
        return lastTagYaw;
    }

    /**
     * Check if we currently have a valid AprilTag lock.
     *
     * @return true if a valid tag is being tracked
     */
    public boolean hasTarget() {
        return hasValidTarget;
    }

    /**
     * Get the ID of the currently tracked AprilTag.
     *
     * @return tag ID, or -1 if no valid target
     */
    public int getTrackedTagId() {
        return hasValidTarget ? lastTrackedTagId : -1;
    }

    /**
     * Get horizontal distance to target in meters.
     * This is the distance in the horizontal plane (X-Z).
     *
     * @return distance in meters, or 0 if no valid target
     */
    public double getDistanceToTargetMeters() {
        return lastDistanceMeters;
    }

    /**
     * Get vertical distance to target in meters.
     * Positive = target is above camera
     * Negative = target is below camera
     *
     * @return height difference in meters, or 0 if no valid target
     */
    public double getHeightToTargetMeters() {
        return lastHeightMeters;
    }

    /**
     * Convert distance from meters to inches.
     *
     * @param meters distance in meters
     * @return distance in inches
     */
    public static double metersToInches(double meters) {
        return meters * 39.3701;
    }

    /**
     * Get horizontal distance in inches.
     *
     * @return distance in inches, or 0 if no valid target
     */
    public double getDistanceToTargetInches() {
        return metersToInches(lastDistanceMeters);
    }

    /**
     * Get vertical distance in inches.
     *
     * @return height in inches, or 0 if no valid target
     */
    public double getHeightToTargetInches() {
        return metersToInches(lastHeightMeters);
    }

    /**
     * Get horizontal angle offset (for aiming).
     * Positive = target to the right
     * Negative = target to the left
     *
     * @return angle in degrees (-29.8 to 29.8)
     */
    public double getTx() {
        return lastTx;
    }

    /**
     * Get vertical angle offset.
     * Positive = target above crosshair
     * Negative = target below crosshair
     *
     * @return angle in degrees (-24.85 to 24.85)
     */
    public double getTy() {
        return lastTy;
    }

    /**
     * Check if target is horizontally centered (within tolerance).
     *
     * @param tolerance tolerance in degrees
     * @return true if aligned
     */
    public boolean isHorizontallyAligned(double tolerance) {
        return Math.abs(lastTx) <= tolerance;
    }

    /**
     * Check if target is vertically centered (within tolerance).
     *
     * @param tolerance tolerance in degrees
     * @return true if aligned
     */
    public boolean isVerticallyAligned(double tolerance) {
        return Math.abs(lastTy) <= tolerance;
    }

    /**
     * Switch Limelight pipeline.
     *
     * @param pipeline pipeline number (0-9)
     */
    public void setPipeline(int pipeline) {
        if (limelight != null) {
            limelight.pipelineSwitch(pipeline);
        }
    }

    /**
     * Check if Limelight hardware is connected.
     *
     * @return true if Limelight is available and connected
     */
    public boolean isConnected() {
        return limelight != null && limelight.isConnected();
    }

    /**
     * Get a formatted status string for telemetry display.
     * One-liner with all important info.
     *
     * @return formatted status string
     */
    public String getStatusString() {
        if (limelight == null) {
            return "Limelight: Not connected";
        }
        if (!hasValidTarget) {
            return "Limelight: No target";
        }
        return String.format(Locale.US, "Tag %d | Distance: %.2fm | Height: %.2fm | tx: %.1f°",
                lastTrackedTagId, lastDistanceMeters, lastHeightMeters, lastTx);
    }

    /**
     * Get the underlying Limelight3A object for direct access if needed.
     *
     * @return Limelight3A instance
     */
    public Limelight3A getLimelight() {
        return limelight;
    }

    /**
     * Stop Limelight processing.
     */
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}