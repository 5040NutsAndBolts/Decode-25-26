package org.firstinspires.ftc.teamcode.helpers.exCam;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;


@TeleOp (name = "AtagCam", group = "Teleop")
public class AtagCamera extends LinearOpMode
{
    OpenCvWebcam camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;


    static final double FEET_PER_METER = 3.28084;


    // Lens stuff
    // UNITS ARE PIXELS
    //this is a rough estimate I made for the ardu 8MP autofocus camera.
    //calibrate properly when time allows
    double fx = 2821;
    double fy = 2116;
    double cx = 1640;
    double cy = 1232;
    //https://en.wikipedia.org/wiki/Camera_resectioning?scrlybrkr=a28d1904 (if you're curious about these calculations and intrisic matricies)

    // UNITS ARE METERS
    double tagsize = 0.166;


    //Tag IDs on Obelisk and Goals
    int gpp = 21;
    int pgp = 22;
    int ppg = 23;
    int red = 24;
    int blue = 20;

    AprilTagDetection tagOfInterest = null;


    @Override
    public void runOpMode() {


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "AprilTag"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);


        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,600, OpenCvCameraRotation.UPRIGHT);
                telemetry.addLine("Camera sucessfully opened");
            }


            @Override
            public void onError(int errorCode) {


            }
        });


        telemetry.setMsTransmissionInterval(50);


        while (!isStarted() && !isStopRequested()) {


            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();


            if(currentDetections.size() != 0) {


                boolean tagFound = false;


                for(AprilTagDetection tag : currentDetections) {


                    if(tag.id == gpp || tag.id == pgp || tag.id == ppg || tag.id == red || tag.id == blue) {


                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }


                if(tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else {
                    telemetry.addLine("Don't see tag of interest :(");


                    if(tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }


            }
            else {
                telemetry.addLine("Don't see tag of interest :(");


                if(tagOfInterest == null) {


                    telemetry.addLine("(The tag has never been seen)");
                }
                else {
                    telemetry.addLine("\nSaw it somewhere :");
                    tagToTelemetry(tagOfInterest);
                }


            }


            telemetry.update();
            sleep(20);
        }


        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */


        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else {
            telemetry.addLine("Never saw :(");
            telemetry.update();
        }


        /* Actually do something useful */
        if(tagOfInterest == null) {
            telemetry.addLine("Nothin there");
        }
        else {
            if(tagOfInterest.id == gpp)
                telemetry.addLine("Green Purple Purple");
            if(tagOfInterest.id == pgp)
                telemetry.addLine("Purple Green Purple");
            if(tagOfInterest.id == ppg)
                telemetry.addLine("Purple Purple Green");
            if(tagOfInterest.id == red)
                telemetry.addLine("Red Goal");
            if(tagOfInterest.id == blue)
                telemetry.addLine("Blue Goal");
        }
    }


    void tagToTelemetry(AprilTagDetection detection)
    {
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);


        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
    }
}
