package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.helpers.PID;
import org.firstinspires.ftc.teamcode.helpers.odo.Odometry;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Drivetrain {
    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;
    public final Odometry odo;
    private double speed = 1;
    private final VoltageSensor voltageSensor;

    public Drivetrain(@NonNull HardwareMap hardwareMap) {
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        frontLeft  = hardwareMap.get(DcMotorEx.class, "Front Left");
        frontRight = hardwareMap.get(DcMotorEx.class, "Front Right");
        backLeft   = hardwareMap.get(DcMotorEx.class, "Back Left");
        backRight  = hardwareMap.get(DcMotorEx.class, "Back Right");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        odo = new Odometry(hardwareMap, 60.0f, 147.32f);
        this.resetOdo();
    }

    public void updateOdo() {
        odo.update();
    }

    /**
     * Returns the shortest signed angular difference from current to target,
     * always in the range (-180, 180]. This prevents the PID from trying to
     * rotate 270° the wrong way when it could rotate 90° the right way.
     */
    public static double angleDifference(double target, double current) {
        double diff = (target - current) % 360.0;
        if (diff > 180.0)  diff -= 360.0;
        if (diff <= -180.0) diff += 360.0;
        return diff;
    }

    /**
     * Drives the robot in its own reference frame.
     *
     * @param forward   forward power (negative for backwards)
     * @param sideways  sideways power (negative for left)
     * @param rotation  rotation power (negative for counterclockwise)
     */
    public void robotOrientedDrive(double forward, double sideways, double rotation) {
        try {
            updateOdo();
        } catch (Exception ignored) {}

        forward  *= speed;
        sideways *= speed;
        rotation *= speed;

        // Scale all inputs proportionally so the largest magnitude is at most 1.
        // This preserves the intended direction even when inputs would sum above 1.
        double scale = Math.abs(rotation) + Math.abs(forward) + Math.abs(sideways);
        if (scale > 1) {
            forward  /= scale;
            rotation /= scale;
            sideways /= scale;
        }

        frontLeft.setPower(forward  - rotation - sideways);
        backLeft.setPower(forward   - rotation + sideways);
        frontRight.setPower(forward + rotation + sideways);
        backRight.setPower(forward  + rotation - sideways);
    }

    /**
     * Drives the robot relative to the field, compensating for the robot's
     * current heading so that "forward" always means the same field direction
     * regardless of which way the robot is facing.
     *
     * The heading is used raw (no normalization) because Math.cos and Math.sin
     * are periodic and handle any angle correctly. Normalizing to [0,360) was
     * previously causing subtle bugs when angles wrapped around.
     *
     * The heading is negated because the odometry reports counterclockwise as
     * positive, but the rotation matrix needs a clockwise-positive convention
     * to correctly transform field vectors into robot vectors.
     */
    public void fieldOrientedDrive(double forward, double sideways, double rotation) {
        odo.update();

        // Use raw heading directly — no normalization needed
        double currentAngle = Math.toRadians(-odo.getPinpoint().getHeading());

        // Standard 2D rotation matrix: rotates the field-frame vector
        // into the robot's own reference frame
        double rotatedForward  = forward  * Math.cos(currentAngle) + sideways * Math.sin(currentAngle);
        double rotatedSideways = -forward * Math.sin(currentAngle) + sideways * Math.cos(currentAngle);

        robotOrientedDrive(rotatedForward, rotatedSideways, rotation);
    }

    public void resetOdo() { odo.reset(); }

    public void neutral() {
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    boolean lastButton = false;

    public void toggleSlowMode(boolean input) {
        if (lastButton != input && input)
            speed = speed == .5 ? 1 : .5;
        lastButton = input;
    }

    public boolean isSlow() { return speed == .5; }

    public double[] getPosition() {
        this.updateOdo(); // ensures position is fresh before returning
        return odo.getPosition();
    }

    public Pose2D getPositionAsPose() {
        return odo.getPinpoint().getPosition();
    }

    /**
     * FIX: updateOdo() is now called before reading velocity, so the returned
     * values are never one cycle stale. Previously getVelocity() read directly
     * from the Pinpoint without updating, causing zero-velocity readings during
     * active motion.
     */
    public double[] getVelocity() {
        this.updateOdo(); // was missing before — caused stale zero velocity reads
        return new double[] {
                odo.getPinpoint().getVelocity().getX(DistanceUnit.INCH),
                odo.getPinpoint().getVelocity().getY(DistanceUnit.INCH),
                Math.toDegrees(odo.getPinpoint().getVelocity().getHeading(AngleUnit.DEGREES))
        };
    }

    @NonNull
    @Override
    public String toString() {
        return  "Slow: " + isSlow() + "\n" +
                "Front Left: "  + frontLeft.getPower()  + "\n" +
                "Front Right: " + frontRight.getPower() + "\n" +
                "Back Left: "   + backLeft.getPower()   + "\n" +
                "Back Right: "  + backRight.getPower()  + "\n" +
                odo.toString()  + "\n" +
                "Battery Voltage: " + voltageSensor.getVoltage();
    }
}