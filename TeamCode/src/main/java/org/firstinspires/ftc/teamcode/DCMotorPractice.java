package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.DCMotorsTest;

@TeleOp
public class DCMotorPractice extends OpMode {
    DCMotorsTest aMotor = new DCMotorsTest();
    @Override
    public void init() {
        aMotor.init(hardwareMap);
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            aMotor.setMotorSpeed(0.5);
        }else {
            aMotor.setMotorSpeed(0);
        }
    }
}
