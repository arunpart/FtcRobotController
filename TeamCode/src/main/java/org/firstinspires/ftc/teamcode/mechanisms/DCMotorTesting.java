package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DCMotorTesting {
    DcMotor aDcMotor;

    public void init(HardwareMap hwMap){
        aDcMotor = hwMap.get(DcMotor.class, "test_motor");
        aDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        aDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        aDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setaDcMotor(double speed){
        aDcMotor.setPower(speed);
    }

}
