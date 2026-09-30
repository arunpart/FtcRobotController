package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DCMotorsTest {

    private DcMotor aLeftMotor, aRightMotor;
    private double ticsPerRev;

    public void init(HardwareMap hwMap) {
        aLeftMotor = hwMap.get(DcMotor.class, "left_drive");
        aRightMotor = hwMap.get(DcMotor.class, "right_drive");

        aLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        aRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Stop immediately at zero power instead of coasting
        aLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        aRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ticsPerRev = aLeftMotor.getMotorType().getTicksPerRev();
    }

    /**
     * Set both motors to the same speed (legacy method)
     */
    public void setMotorSpeed(double speed) {
        aLeftMotor.setPower(speed);
        aRightMotor.setPower(speed);
    }

    /**
     * Set left motor speed individually
     */
    public void setLeftMotorSpeed(double speed) {
        aLeftMotor.setPower(speed);
    }

    /**
     * Set right motor speed individually
     */
    public void setRightMotorSpeed(double speed) {
        aRightMotor.setPower(speed);
    }

    /**
     * Set both motors with individual speeds (arcade drive)
     */
    public void setMotorSpeeds(double leftSpeed, double rightSpeed) {
        aLeftMotor.setPower(leftSpeed/4);
        aRightMotor.setPower(rightSpeed/4);
    }

    /**
     * Get left motor revolutions
     */
    public double getLeftMotorsRev() {
        return aLeftMotor.getCurrentPosition() / ticsPerRev;
    }

    public double getLeftTics() {
        return aLeftMotor.getMotorType().getTicksPerRev();
    }

    public double getRightTics() {
        return aRightMotor.getCurrentPosition();
    }

    /**
     * Get right motor revolutions
     */
    public double getRightMotorsRev() {
        return aRightMotor.getCurrentPosition() / ticsPerRev;
    }

    /**
     * Stop both motors
     */
    public void stopMotors() {
        aLeftMotor.setPower(0);
        aRightMotor.setPower(0);
    }
}
