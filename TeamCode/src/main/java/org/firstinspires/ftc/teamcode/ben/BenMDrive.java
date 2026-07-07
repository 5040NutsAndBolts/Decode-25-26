package org.firstinspires.ftc.teamcode.ben;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="BenMDrive", group="Teleop")
public class BenMDrive extends OpMode {
    private BenMTrain bt;
    @Override
    public void init(){
        bt = new BenMTrain(hardwareMap);
    }
    @Override
    public void loop(){
        bt.driving(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
    }
}