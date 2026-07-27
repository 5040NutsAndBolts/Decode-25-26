package org.firstinspires.ftc.teamcode.ben;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.helpers.PID;

import java.util.HashMap;

@TeleOp(name="BenFlywheelPID", group="Teleop")
public class BenFlywheelPID extends OpMode {
    FtcDashboard dash;
    TelemetryPacket packet;
    PID speedControl;
    private DcMotorEx flywheel;
    public void init()
    {
        speedControl = new PID(0.1,0,0, this::flywheelRPMS, 0);
        flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        dash = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
    }

    public void loop()
    {
        speedControl.setTarget(4000);
        if(gamepad1.x) {
            flywheel.setPower(speedControl.autoControl());
            getPIDTelemetry();
        }
    }
    public double flywheelRPMS() {
        double currentTPS = flywheel.getVelocity();
        return (currentTPS * (2.67857485));
    }

    public void getPIDTelemetry() {
        packet.clearLines();
        HashMap<String, Object> map = new HashMap<>();
        map.put("Flywheel RPMS", flywheelRPMS());
        packet.putAll(map);
        dash.sendTelemetryPacket(packet);
        packet.clearLines();
    }
}
