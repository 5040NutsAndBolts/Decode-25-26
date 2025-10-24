package org.firstinspires.ftc.teamcode.helpers.camera;

import android.util.Size;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.helpers.camera.exCam.colorDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

public class ColorLocate {
    private final VisionPortal vPortal;

    public ColorLocate(@NonNull HardwareMap hardwareMap) {

        colorDetectionPipeline cPipe = new colorDetectionPipeline();

        vPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "color"))
                .addProcessor(cPipe)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setCameraResolution(new Size(800,600))
                .enableLiveView(true)
                .build();
    }

    public void close() {
        if (vPortal != null) {
            vPortal.close();
        }
    }

    public void check(){ //only important if using multiple cameras
        vPortal.getActiveCamera();
    }

    public String currState(){
        return "" + vPortal.getCameraState();
    }

}
