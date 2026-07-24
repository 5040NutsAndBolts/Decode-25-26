package org.firstinspires.ftc.teamcode.ben;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BenMTrain {

    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;

    public BenMTrain(@NonNull HardwareMap hardwareMap) {

        frontLeft = hardwareMap.get(DcMotorEx.class, "Front Left");
        frontRight = hardwareMap.get(DcMotorEx.class, "Front Right");
        backLeft = hardwareMap.get(DcMotorEx.class, "Back Left");
        backRight = hardwareMap.get(DcMotorEx.class, "Back Right");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void driving(double forward, double sideways, double rotation)
    {
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
}
