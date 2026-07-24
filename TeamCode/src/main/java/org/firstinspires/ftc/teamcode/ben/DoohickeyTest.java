package org.firstinspires.ftc.teamcode.ben;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="DoohickeyTest", group="Teleop")
public class DoohickeyTest extends OpMode {
    private Doohickey doohickey;

    @Override
    public void init(){
        doohickey = new Doohickey(hardwareMap);
    }
    @Override
    public void loop() {
        doohickey.does(gamepad1.left_stick_y);
        if (Math.abs(gamepad1.right_stick_x) >= 0.5) {
            doohickey.flip();
        }
    }

}
