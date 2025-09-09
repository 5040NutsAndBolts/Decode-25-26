package org.firstinspires.ftc.teamcode.helpers.odo;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.helpers.odo.GoBildaPinpointDriver;

public class Odometry {
    private final GoBildaPinpointDriver pinpoint;

    public Odometry (@NonNull HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        //We use the swingarm GoBilda pods, change if you are using different pods (it's ticks/mm)
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);

        //Depends on mounting of pods
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        //THIS WILL CHANGE WITH EVERY DRIVETRAIN!
        //Offsets of each pod from the center of the robot (in mm)
        pinpoint.setOffsets(3,2);

        //Recalibrate IMU
        pinpoint.resetPosAndIMU();

        //Ensure that we start  at 0,0
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,0,0, AngleUnit.DEGREES,0));
    }

    public Pose2D getPosition () {
        return pinpoint.getPosition();
    }

    public GoBildaPinpointDriver getPinpoint() {
    	return pinpoint;
    }

    public void update() {
        pinpoint.update();
    }

    public void reset() {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,0,0, AngleUnit.DEGREES,0));
    }
    public void reset(int x, int y, int heading) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,x,y, AngleUnit.DEGREES,heading));
    }

    @NonNull
    @Override
    public String toString() {
        return "Odometry:\n" +
                "Status: " + pinpoint.getDeviceStatus() + "\n" +
                "X: " + (pinpoint.getPosX() * .0393700787) + "\n" +
                "Y: " + (pinpoint.getPosY() * .0393700787) + "\n" +
                "Heading: " + Math.toDegrees(pinpoint.getHeading());
    }
}