package org.firstinspires.ftc.teamcode.ben;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="BenDrive", group="Teleop")
public class BenDrive extends OpMode {
    private BenTrain bt;
    @Override
    public void init(){
        bt = new BenTrain(hardwareMap);
    }
    @Override
    public void loop(){
        bt.driving(gamepad1.left_stick_y,gamepad1.right_stick_x);
    }
}
