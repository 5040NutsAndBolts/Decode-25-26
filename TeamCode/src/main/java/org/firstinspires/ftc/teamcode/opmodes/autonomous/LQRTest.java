package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.helpers.LQR;
import org.firstinspires.ftc.teamcode.helpers.PID;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

@TeleOp(name = "LQRTest", group = "Teleop")
public class LQRTest extends OpMode {
    private Drivetrain dt;
    FtcDashboard dash;
    TelemetryPacket packet;
    Launcher la;
    PID fC;
    LQR lqr;


    private void sendTelemetry(String loopName, Map<String, Object> additional) {
        packet.clearLines();
        HashMap<String, Object> map = new HashMap<>(additional);
        map.put("Name", loopName);
        map.put("Pos", Arrays.toString(dt.getPosition()));
        map.put("RPMS: ", la.flywheelRPMS());
        map.put("RPM Target: ", fC.getTarget());
        packet.putAll(map);
        dash.sendTelemetryPacket(packet);
        packet.clearLines();
    }

    private void sendTelemetry(String loopName) {
        packet = new TelemetryPacket();
        HashMap<String, Object> map = new HashMap<>();
        map.put("Name", loopName);
        map.put("Pos", Arrays.toString(dt.getPosition()));
        map.put("RPMS: ", la.flywheelRPMS());
        map.put("RPM Target: ", fC.getTarget());
        packet.putAll(map);
        dash.sendTelemetryPacket(packet);
        packet.clearLines();
    }

    @Override
    public void init() {
        dt = new Drivetrain(hardwareMap);
        la = new Launcher(hardwareMap);
        fC = new PID(.0075, 3e-8, 0, la::flywheelRPMS, 0);
        lqr = new LQR();

        packet = new TelemetryPacket();
        dash = FtcDashboard.getInstance();

        sendTelemetry("Initialization");
        dt.resetOdo();

    }

    @Override
    public void loop() {


        lqr.updateLQR(dt.getPosition()[0], dt.getPosition()[1], dt.getPosition()[2], dt.getVelocity()[0], dt.getVelocity()[1], dt.getVelocity()[2]);

        SimpleMatrix powers = lqr.calculate(lqr.state);

        dt.directDrive(powers.get(0,0), powers.get(1,0), powers.get(2,0), powers.get(3,0));

        if(gamepad1.a)
            lqr.setTarget(24,0,0);


        la.setOuttakePower(0);
        dt.neutral();
        dt.updateOdo();
        }
    }


