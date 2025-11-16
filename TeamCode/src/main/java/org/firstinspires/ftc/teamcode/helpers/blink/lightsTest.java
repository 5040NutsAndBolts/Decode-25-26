package org.firstinspires.ftc.teamcode.helpers.blink; // Make sure this package is correct for your project

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="lights test", group="Teleop")
public class lightsTest extends OpMode {

    // Declare your helper object
    private BlinkIn lights;

    /**
     * This code runs ONCE when you press INIT on the Driver Station.
     */
    @Override
    public void init() {
        // Initialize your BlinkIn helper class
        // This will find the "lights" in the hardwareMap
        // and call resetDeviceConfigurationForOpMode()
        try {
            lights = new BlinkIn(hardwareMap);
            telemetry.addData("Status", "BlinkIn Initialized");
        } catch (Exception e) {
            telemetry.addData("Status", "Error Initializing BlinkIn: " + e.getMessage());
        }

        telemetry.addData("Controls", "Press A, B, X, Y to change colors");
        telemetry.update();
    }

    /**
     * This code runs REPEATEDLY after you press PLAY.
     */
    @Override
    public void loop() {

        // Check which button is pressed on gamepad 1
        // and call the correct method from your BlinkIn class.

        if (gamepad1.a) {
            // If 'A' is pressed, turn lights blue
            lights.turnBlue();
            telemetry.addData("Color", "Blue (A pressed)");
        } else if (gamepad1.b) {
            // If 'B' is pressed, turn lights red
            lights.turnRed();
            telemetry.addData("Color", "Red (B pressed)");
        } else if (gamepad1.x) {
            // If 'X' is pressed, turn lights yellow
            lights.turnYellow();
            telemetry.addData("Color", "Yellow (X pressed)");
        } else if (gamepad1.y) {
            // If 'Y' is pressed, turn lights orange
            lights.turnOrange();
            telemetry.addData("Color", "Orange (Y pressed)");
        }

        // Update the telemetry on the Driver Station
        telemetry.update();
    }
}