package org.firstinspires.ftc.teamcode.ben;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Doohickey {
    private final CRServo dooServo;

    public Doohickey(@NonNull HardwareMap hardwareMap)
    {
        dooServo = hardwareMap.get(CRServo.class, "Servo");
    }

    public void does(double doing)
    {
        dooServo.setPower(doing);
    }

    public void flip()
    {
        dooServo.setDirection(dooServo.getDirection().inverted());
    }


}
