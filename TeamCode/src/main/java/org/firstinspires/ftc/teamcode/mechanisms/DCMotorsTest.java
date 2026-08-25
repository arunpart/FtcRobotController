package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DCMotorsTest {

    private DcMotor aLeftMotor, aRightMotor;
    private double  ticsPerRev;

    public void init(HardwareMap hwMap){
        aLeftMotor = hwMap.get(DcMotor.class, "left_drive");
        aRightMotor = hwMap.get(DcMotor.class, "right_drive");
        //aRightMotor = hwMap.get(DcMotor.class, "right_drive");
        aLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        aRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ticsPerRev = aLeftMotor.getMotorType().getTicksPerRev();
    }

    public void setMotorSpeed(double speed){
        aLeftMotor.setPower(speed);
        aRightMotor.setPower(speed);

    }

    public double getLeftMotorsRev() {
        return aLeftMotor.getCurrentPosition() / ticsPerRev;
    }
}
