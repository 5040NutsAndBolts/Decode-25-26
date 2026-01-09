
package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lights {
    private final Servo lights;
    public Lights(@NonNull HardwareMap hardwareMap, String name) {
        this.lights = hardwareMap.get(Servo.class, name);
    }

    public enum Color {
        OFF(0),
        RED(.277),
        BLOOD_ORANGE(0.305),
        ORANGE(.333),
        GREEN(.5),
        BLUE(.611),
        PURPLE(.722),
        YELLOW(.388),
        WHITE(1);

        private final double value;
        Color(double value) {
            this.value = value;
        }
    }

    public void setPattern(@NonNull Color pattern) {
        lights.setPosition(pattern.value);
    }
}

