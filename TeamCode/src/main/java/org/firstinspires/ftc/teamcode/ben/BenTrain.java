package org.firstinspires.ftc.teamcode.ben;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class BenTrain {

   private final DcMotorEx frontLeft, frontRight, backLeft, backRight;
   public BenTrain(@NonNull HardwareMap hardwareMap) {

       frontLeft = hardwareMap.get(DcMotorEx.class, "Front Left");
       frontRight = hardwareMap.get(DcMotorEx.class, "Front Right");
       backLeft = hardwareMap.get(DcMotorEx.class, "Back Left");
       backRight = hardwareMap.get(DcMotorEx.class, "Back Right");

   }

   public void driving(double forward, double rotation)
   {
       double scale = Math.abs(rotation) + Math.abs(forward);
       if (scale > 1) {
           forward  /= scale;
           rotation /= scale;
       }

       frontLeft.setPower(forward  - rotation);
       backLeft.setPower(forward   - rotation);
       frontRight.setPower(forward + rotation);
       backRight.setPower(forward  + rotation);
   }

}
