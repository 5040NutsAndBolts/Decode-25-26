package org.firstinspires.ftc.teamcode.helpers.blink;
import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class BlinkIn {
    private final RevBlinkinLedDriver lights;

    public BlinkIn(@NonNull HardwareMap hardwareMap){
        // Get the lights from the hardware map
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");

        // --- IMPROVEMENT ---
        // Reset the device configuration just once when it's created.
        lights.resetDeviceConfigurationForOpMode();
    }

    public void turnRed(){
        // Now you just need to set the pattern!
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
    }

    public void turnBlue(){
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
    }

    public void turnYellow(){
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
    }

    public void turnOrange(){
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
    }
}