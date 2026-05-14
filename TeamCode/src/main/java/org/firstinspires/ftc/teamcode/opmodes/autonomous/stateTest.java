package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.LQR;
import org.firstinspires.ftc.teamcode.helpers.PID;
import org.firstinspires.ftc.teamcode.helpers.States;
import org.firstinspires.ftc.teamcode.mechanisms.CSensor;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.mechanisms.Lights;

@TeleOp(name = "MyTeleop")
public class stateTest extends OpMode {
    Drivetrain dt;
    Launcher la;
    States fsm;
    CSensor colorSensor;

    @Override
    public void init() {
        dt = new Drivetrain(hardwareMap);
        la = new Launcher(hardwareMap);
        fsm = new States(la,colorSensor);

        dt.resetOdo();

    }

    @Override
    public void loop() {
        fsm.transition(gamepad1.a);

        telemetry.addData("State", fsm.getState());

    }
}
