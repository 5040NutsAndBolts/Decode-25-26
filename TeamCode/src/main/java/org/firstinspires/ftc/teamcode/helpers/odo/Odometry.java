package org.firstinspires.ftc.teamcode.helpers.odo;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.helpers.odo.GoBildaPinpointDriver;

public class Odometry {
    private final GoBildaPinpointDriver pinpoint;

    public Odometry (@NonNull HardwareMap hardwareMap, float xOffset, float yOffset) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        //We use the swingarm GoBilda pods, change if you are using different pods (it's ticks/mm)
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);

        //Depends on mounting of pods
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        //THIS WILL CHANGE WITH EVERY DRIVETRAIN!
        //Offsets of each pod from the center of the robot (in mm)
        pinpoint.setOffsets(xOffset,yOffset);

        //Recalibrate IMU
        pinpoint.resetPosAndIMU();
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
                "\tStatus: " + pinpoint.getDeviceStatus() + "\n" +
                "\tX: " + (pinpoint.getPosition().getX(DistanceUnit.INCH)) + "\n" +
                "\tY: " + (pinpoint.getPosition().getY(DistanceUnit.INCH)) + "\n" +
                "\tHeading: " + Math.toDegrees(pinpoint.getHeading());
    }
}