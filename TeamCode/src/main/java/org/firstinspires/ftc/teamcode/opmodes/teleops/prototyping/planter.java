package org.firstinspires.ftc.teamcode.opmodes.teleops.prototyping;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="SirPlantsAlot", group="Teleop")
public class planter extends OpMode {
    private Servo plant;
    @Override
    public void init() {

        plant = hardwareMap.get(Servo.class,"planter");
    }

    @Override
    public void loop() {
        if(gamepad1.a)
            plant.setPosition(0);
        if(gamepad1.b)
            plant.setPosition(1);
    }
}
