package org.firstinspires.ftc.teamcode.ben;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Bendrive", group="Teleop")
public class Bendrive extends OpMode {
    private Bentrain bt;
    @Override
    public void init(){
        bt = new Bentrain(hardwareMap);
    }
    @Override
    public void loop(){
        bt.driving(gamepad1.left_stick_y,gamepad1.right_stick_x);
    }
}
