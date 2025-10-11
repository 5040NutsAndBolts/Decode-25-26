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
import org.firstinspires.ftc.teamcode.helpers.easypathing.Mechanism;

public class Drivetrain extends Mechanism {
    public boolean waitWorthy = true;
    private final DcMotorEx frontLeft,frontRight,backLeft,backRight;
    private final Odometry odo;
    private double speed = 1;

    public Drivetrain(@NonNull HardwareMap hardwareMap) {
        //Drive motor initialization
        frontLeft = hardwareMap.get(DcMotorEx.class, "Front Left");
        frontRight = hardwareMap.get(DcMotorEx.class, "Front Right");
        backLeft = hardwareMap.get(DcMotorEx.class, "Back Left");
        backRight = hardwareMap.get(DcMotorEx.class, "Back Right");

        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //TODO: CONFIGURE OFFSETS
        odo = new Odometry(hardwareMap, 121.92f, 147.32f);
    }

    /**
     * Drives robot based off current position of robot, not field
     * @param forward forward power (negative for backwards)
     * @param sideways sideways power (negative for left)
     * @param rotation rotation power (negative for counterclockwise)
     */
    public void robotOrientedDrive(double forward, double sideways, double rotation) {
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
    public void fieldOrientedDrive(double forward, double sideways, double rotation, @NonNull Odometry odo) {
        double P = Math.hypot(sideways, forward);
        double currentAngle = odo.getPinpoint().getHeading();

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

    /**
     * toggles slow mode
     * @return slow mode is true
     */
    public boolean isSlow() {return speed == .5;}

    //TODO: CONFIGURE PIDS
    private final PID xpid = new PID(.15,0,0);
    private final PID ypid = new PID(.15,0,0);
    private final PID rpid = new PID(.15,0,0);

    /**
     * moves the drivetrain to a desired position
     * @param pos Doubles, [x,y,r] in inches and degrees
     */
    public void update (@NonNull Object[] pos) {
        for(Object o : pos)
            assert (o instanceof Double || o instanceof Float || o instanceof Integer) && pos.length == 3;
        xpid.setTarget((Double) pos[0]);
        ypid.setTarget((Double) pos[1]);
        rpid.setTarget((Double) pos[2]);

        odo.update();
        fieldOrientedDrive(
                ypid.autoControl(odo.getPosition().getX(DistanceUnit.INCH)),
                xpid.autoControl(odo.getPosition().getY(DistanceUnit.INCH)),
                rpid.autoControl(odo.getPosition().getHeading(AngleUnit.DEGREES)),
                odo
        );
    }

    /**
     * is the robot within the tolerance of the target (for easypathing)
     * @return true if the robot is within the tolerance of the target
     */
	public boolean isFinished(@NonNull Object[] tolerances) {
        for(Object o : tolerances)
            assert (o instanceof Double || o instanceof Float || o instanceof Integer) && tolerances.length == 2;
		double rotMOE = (double) tolerances[0];
		double xyRMOE = (double) tolerances[1];
		return Math.hypot(xpid.getTarget() - odo.getPosition().getX(DistanceUnit.INCH), ypid.getTarget() - odo.getPosition().getY(DistanceUnit.INCH)) < xyRMOE &&
                  Math.abs(rpid.getTarget() - odo.getPosition().getHeading(AngleUnit.DEGREES)) < rotMOE;
    }

    /**
     * holds the robot at the current position
     */
    Pose2D pos;
    private boolean lastState;
    public void hold(boolean in) {
        if (in) {
            if(in != lastState)
                pos = odo.getPosition();
            this.update(new Object[]{
                pos.getX(DistanceUnit.INCH),
                pos.getY(DistanceUnit.INCH),
                pos.getHeading(AngleUnit.DEGREES)
            });
        }else pos = null;
        lastState = in;
    }

    @NonNull
    @Override
    public String toString() {
        return
                "Front Left: " + frontLeft.getPower() + "\n" +
                "Front Right: " + frontRight.getPower() + "\n" +
                "Back Left: " + backLeft.getPower() + "\n" +
                "Back Right: " + backRight.getPower() + "\n" +
                "X: " + odo.getPosition().getX(DistanceUnit.INCH) + "\n" +
                "Y: " + odo.getPosition().getY(DistanceUnit.INCH) + "\n" +
                "Rotation: " + odo.getPosition().getHeading(AngleUnit.DEGREES) + "\n" +
                "X controller: " + xpid + "\n" +
                "Y controller: " + ypid + "\n" +
                "Rotation controller: " + rpid;
    }
}