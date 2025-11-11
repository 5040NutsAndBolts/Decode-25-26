package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo Test", group="Teleop")
public class ServoTest extends OpMode {
    private Servo shmegus;
    private CRServo shmogus;
    @Override
    public void init() {

        shmegus = hardwareMap.get(Servo.class,"servo");
        shmogus = hardwareMap.get(CRServo.class,"CRservo");
    }

    @Override
    public void loop() {
        if(gamepad2.a)
            shmegus.setPosition(0);
        if(gamepad2.b)
            shmegus.setPosition(1);
        if(gamepad1.left_stick_y > 0.5)
            shmogus.setPower(1);
        else shmogus.setPower(0);
    }
}
