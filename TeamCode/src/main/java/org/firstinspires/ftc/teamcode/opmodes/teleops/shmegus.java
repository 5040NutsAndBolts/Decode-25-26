package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="shmegus", group="Teleop")
public class shmegus extends OpMode {
    private Servo shmegus;
    @Override
    public void init() {

        shmegus = hardwareMap.get(Servo.class,"servo");
    }

    @Override
    public void loop() {
        if(gamepad1.a)
            shmegus.setPosition(0);
        if(gamepad1.b)
            shmegus.setPosition(1);
    }
}
