package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class GamepadExample extends OpMode {
    @Override
    public void init() {

    }

    @Override
    public void loop() {
        double speedForward = -gamepad1.right_stick_y;

        telemetry.addData("right x", gamepad1.right_stick_x);
        telemetry.addData("right y", speedForward);
        telemetry.addData("a", gamepad1.a);
    }
}
