package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;

public class CThellyMTP {

    Drivetrain dt;

    /**
        PIDs, not worried about a wrap around bug since
        we normalize the angle.
     */
    PID xController = new PID(0.02,1e-4,0);
    PID yController = new PID(0.02,1e-4,0);
    PID headingController = new PID(0.03,3e-5,0);

    //just a way to organize position and angle in a simple way
    public static class Pose {
        public double x, y, heading;
        public Pose(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }
    /**
     * for finding the shortest path between the angles.
     * without wrapping the error in this way, the robot
     * may spin the wrong way and spend unnecessary time.
     * ex. heading is 350 degrees and target is 10 degrees,
     * rather than going with 10 - 350 = -340 and spinning
     * -340 degrees to the target, it will fix it so that
     * it spins 20 degrees
     */
    double normalizeAngle(double radians){
        while (radians >= Math.PI)
            radians -= 2 * Math.PI;
        while (radians < -Math.PI)
            radians += 2 * Math.PI;
        return radians;
    }

    void moveToPosition(Pose target, double[] errThreshold, double[] maxVel){

        while(!Thread.currentThread().isInterrupted()) {
            Pose currentPos = new Pose(dt.getPosition()[0], dt.getPosition()[1], Math.toRadians(dt.getPosition()[2]));

            xController.setTarget(target.x);
            yController.setTarget(target.y);
            headingController.setTarget(target.heading);

            double deltaX = target.x - currentPos.x;
            double deltaY = target.y - currentPos.y;
            double deltaTheta = normalizeAngle(target.heading - currentPos.heading);

            double xOut = xController.autoControl(currentPos.x);
            double yOut = yController.autoControl(currentPos.y);
            double turnOut = headingController.autoControl(Math.toDegrees(deltaTheta));

            dt.fieldOrientedDrive(xOut,yOut,turnOut);

            if (Math.abs(deltaX) < errThreshold[0] && Math.abs(deltaY) < errThreshold[1] && Math.abs(deltaTheta) < errThreshold[2]) {
                dt.robotOrientedDrive(0, 0, 0);
                break;
            }
        }

    }


}
