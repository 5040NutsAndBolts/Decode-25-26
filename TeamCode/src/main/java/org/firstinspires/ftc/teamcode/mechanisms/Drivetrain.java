package org.firstinspires.ftc.teamcode.mechanisms;
import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.helpers.PID;
import org.firstinspires.ftc.teamcode.helpers.odo.Odometry;
import org.firstinspires.ftc.teamcode.helpers.easypathing.Mechanism;
import com.qualcomm.robotcore.hardware.VoltageSensor;
public class Drivetrain extends Mechanism {
    public boolean waitWorthy = true;
    private final DcMotorEx frontLeft,frontRight,backLeft,backRight;
    private final Odometry odo;
    private double speed = 1;
    private final VoltageSensor voltageSensor;
    //public ColorSensor colorSensor;
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

        //colorSensor = hardwareMap.get(ColorSensor.class, "Color Sensor");

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

    /**
     * toggles slow mode
     * @return slow mode is true
     */
    public boolean isSlow() {return speed == .5;}

    //TODO: CONFIGURE PIDS
    private final PID xpid = new PID(.025,0.000001,0);
    private final PID ypid = new PID(.025,0.000001,0);
    private final PID rpid = new PID(0,0,0);

    /**
     * moves the drivetrain to a desired position
     * @param pos Doubles, [x,y,r] in inches and degrees
     */
    public void update (@NonNull double[] pos) {
        xpid.setTarget(pos[0]);
        ypid.setTarget(pos[1]);
        rpid.setTarget(pos[2]);

        odo.update();
        fieldOrientedDrive(
                ypid.autoControl(odo.getPosition()[0]),
                xpid.autoControl(odo.getPosition()[1]),
                rpid.autoControl(odo.getPosition()[2])
        );
    }

    /**
     * is the robot within the tolerance of the target (for easypathing)
     * @return true if the robot is within the tolerance of the target
     */
	public boolean isFinished(@NonNull double[] tolerances) {
		double rotMOE =  tolerances[0];
		double xyRMOE = tolerances[1];
		return Math.hypot(xpid.getTarget() - odo.getPosition()[0], ypid.getTarget() - odo.getPosition()[1]) < xyRMOE &&
                  Math.abs(rpid.getTarget() - odo.getPosition()[2]) < rotMOE;
    }

    /**
     * holds the robot at the current position
     */
    double[] pos;
    private boolean lastState;
    public void hold(boolean in) {
        if (in) {
            if(in != lastState)
                pos = odo.getPosition();
            this.update(pos);
        }else pos = null;
        lastState = in;
    }

    public double[] getPosition() {
        return odo.getPosition();
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
                "X controller: " + xpid + "\n" +
                "Y controller: " + ypid + "\n" +
                "Rotation controller: " + rpid + "\n" +
                "Battery Voltage: " + voltageSensor.getVoltage();
    }
}