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

import java.util.HashMap;

public class Drivetrain {
    private final DcMotorEx frontLeft,frontRight,backLeft,backRight;
    public final Odometry odo;
    private double speed = 1;
    private final VoltageSensor voltageSensor;
    public Drivetrain(@NonNull HardwareMap hardwareMap) {
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        //Drive motor initialization
        frontLeft = hardwareMap.get(DcMotorEx.class, "Front Left");
        frontRight = hardwareMap.get(DcMotorEx.class, "Front Right");
        backLeft = hardwareMap.get(DcMotorEx.class, "Back Left");
        backRight = hardwareMap.get(DcMotorEx.class, "Back Right");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //TODO: CONFIGURE OFFSETS
        odo = new Odometry(hardwareMap, 121.92f, 147.32f);
        this.resetOdo();
    }


    public void updateOdo() {
        odo.update();
    }

    /**
     * Drives robot based off current position of robot, not field
     * @param forward forward power (negative for backwards)
     * @param sideways sideways power (negative for left)
     * @param rotation rotation power (negative for counterclockwise)
     */
    public void robotOrientedDrive(double forward, double sideways, double rotation) {
        try {
            updateOdo();
        }catch(Exception ignored) {}
        //Multiplied by speed variable, only changes when in slowmode
        forward *= speed;
        sideways *= speed;
        rotation *= speed;

        //adds all the inputs together to get the number to scale it by
        double scale = Math.abs(rotation) + Math.abs(forward) + Math.abs(sideways);

        //Scales the inputs between 0-1 for the setPower() method
        if (scale > 1) {
            forward /= scale;
            rotation /= scale;
            sideways /= scale;
        }

        //Zeroes out and opposing or angular force from the Mecanum wheels
        frontLeft.setPower(forward - rotation - sideways);
        backLeft.setPower(forward - rotation + sideways);
        frontRight.setPower(forward + rotation + sideways);
        backRight.setPower(forward + rotation - sideways);
    }

    //Battery/motor saver
    public void neutral() {
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    //field oriented drive
    public void fieldOrientedDrive(double forward, double sideways, double rotation) {
        odo.update();
        double P = Math.hypot(sideways, forward);
        double currentAngle = (odo.getPinpoint().getHeading() + 360)%360;

        double robotAngle = Math.atan2(forward, sideways);


        double vcos = P * Math.cos(robotAngle - currentAngle);
        double vsin = P * Math.sin(robotAngle - currentAngle);
        double v5 = vsin + vcos - rotation;
        double v6 = vsin - vcos + rotation;
        double v7 = vsin - vcos - rotation;
        double v8 = vsin + vcos + rotation;

        frontLeft.setPower(v5);
        frontRight.setPower(v6);
        backLeft.setPower(v7);
        backRight.setPower(v8);
    }

    public void resetOdo(){odo.reset();}


    boolean lastButton = false;
    /**
     * toggles slow mode
     * @param input gamepad input
     */
    public void toggleSlowMode(boolean input) {
        if(lastButton != input && input)  //If button state has changed and it is pressed
            speed = speed ==.5 ? 1 : .5;
        lastButton = input;
    }


    private final PID xpid = new PID(.025,0.000001,0);
    private final PID ypid = new PID(.025,0.000001,0);
    private final PID rpid = new PID(0,0,0);
    public void setTarget (@NonNull double[] target) {
        xpid.setTarget(target[0]);
        ypid.setTarget(target[1]);
        rpid.setTarget(target[2]);
    }

    public void updateMoveTo () {
        this.fieldOrientedDrive(
                xpid.autoControl(odo.getPosition()[0]),
                ypid.autoControl(odo.getPosition()[1]),
                rpid.autoControl(odo.getPosition()[2])
        );
        this.odo.update();
    }

    public double[] getTargetPosition() {
        return new double[] {
                xpid.getTarget(),
                ypid.getTarget(),
                rpid.getTarget()
        };
    }


    /**
     * toggles slow mode
     * @return slow mode is true
     */
    public boolean isSlow() {return speed == .5;}

    public double[] getPosition() {
        this.updateOdo();
        return odo.getPosition();
    }

    public Pose2D getPositionAsPose() {
        return odo.getPinpoint().getPosition();
    }

    public HashMap <String, Object> getPIDTelemetry () {
        HashMap <String, Object> map = new HashMap<>();
        map.put("Odo X", odo.getPinpoint().getPosition().getX(DistanceUnit.INCH));
        map.put("Odo Y", odo.getPinpoint().getPosition().getY(DistanceUnit.INCH));
        map.put("Odo R", odo.getPinpoint().getPosition().getHeading(AngleUnit.DEGREES));
        map.put("Vel X", odo.getPinpoint().getVelocity().getX(DistanceUnit.INCH));
        map.put("Vel Y", odo.getPinpoint().getVelocity().getY(DistanceUnit.INCH));
        map.put("Vel R", odo.getPinpoint().getVelocity().getHeading(AngleUnit.DEGREES));
        map.put("X controller", xpid.toString());
        map.put("Y controller", ypid.toString());
        map.put("Rotation controller", rpid.toString());
        return map;
    }

    public double[] getVelocity() {
        return new double[] {
                odo.getPinpoint().getVelocity().getX(DistanceUnit.INCH),
                odo.getPinpoint().getVelocity().getY(DistanceUnit.INCH),
                Math.toDegrees(odo.getPinpoint().getVelocity().getHeading(AngleUnit.DEGREES))
        };
    }


    @NonNull
    @Override
    public String toString() {
        return
                "Slow: "+isSlow()+"\n"+
                "Front Left: " + frontLeft.getPower() + "\n" +
                "Front Right: " + frontRight.getPower() + "\n" +
                "Back Left: " + backLeft.getPower() + "\n" +
                "Back Right: " + backRight.getPower() + "\n" +
                odo.toString() + "\n" +
                "Battery Voltage: " + voltageSensor.getVoltage();
    }
}