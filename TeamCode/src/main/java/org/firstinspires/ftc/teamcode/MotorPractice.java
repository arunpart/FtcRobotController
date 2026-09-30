package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.DCMotorTesting;
@TeleOp
public class MotorPractice extends OpMode {
DCMotorTesting aTest = new DCMotorTesting();

boolean aBooleanValueA = gamepad1.a;
double aDoubleSpeed = gamepad1.left_stick_y;

    @Override
    public void init() {
        aTest.init(hardwareMap);
    }

    @Override
    public void loop() {
        if(aBooleanValueA)
            aTest.setaDcMotor(-0.5);
        else{
            aTest.setaDcMotor(aDoubleSpeed);
        }
    }
}
