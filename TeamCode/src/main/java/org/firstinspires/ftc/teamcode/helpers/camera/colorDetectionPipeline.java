package org.firstinspires.ftc.teamcode.helpers.camera;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

public class colorDetectionPipeline extends OpenCvPipeline
{
    public static double height = 0;
    public static double width = 0;
    OpenCvWebcam webcam;

    // Coordinate position of the top left corner of the selected rectangle
    public static Point screenPosition = new Point(0,0);

    private final Mat initialImg, establishedImg, drawingMat, hierarchy;

    //Makes an array to hold all currently seen artifact's contours
    ArrayList<List<MatOfPoint>> establishSets = new ArrayList<>();

    /**
     * Sets up all the variables to keep code clean
     */
    public colorDetectionPipeline(@NonNull HardwareMap hardwareMap) {
        initialImg = new Mat();
        establishedImg = new Mat();
        drawingMat = new Mat();
        hierarchy = new Mat();



        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                //set this line to dimensions of webcam
                webcam.startStreaming(800, 600, OpenCvCameraRotation.UPRIGHT);
                //cheap logitechs are 320, 240
                //logitech brio is 1280, 720
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        webcam.setPipeline(this);
    }



    public OpenCvWebcam getStream () {
        return webcam;
    }


    @Override
    public Mat processFrame(@NonNull Mat input) {
        // Copies the original input to other materials to be worked on so they aren't overriding each other
        input.copyTo(initialImg);
        input.copyTo(establishedImg);


        //converts the image from rgb to hsv
        Imgproc.cvtColor(initialImg, establishedImg,Imgproc.COLOR_RGB2HSV);

        //controls the color range the camera is looking for in the hsv color space
        //the hue value is scaled by .5, the saturation and value are scaled by 2.55
        //needs to be changed to fit the new purple and green colors!!!!!!!!!!!!!!!!!!!!!!!1
            Core.inRange(establishedImg,new Scalar(112,180,150),new Scalar(125,255,255),establishedImg);

            Core.inRange(establishedImg,new Scalar(0,150,100),new Scalar(255,255,255),establishedImg);


        // Creates a list for all contoured objects the camera will find
        List<MatOfPoint> contoursList = new ArrayList<>();

        Imgproc.findContours(establishedImg, contoursList, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        Imgproc.drawContours(drawingMat, contoursList,-1, new Scalar(40,40,40),2);

        // Scores all the contours and selects the best of them
        for(MatOfPoint contour : contoursList) {

            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(contour);

            // Draw the current found rectangle on the selections mask
            //     Drawn in blue
            Imgproc.rectangle(drawingMat, rect.tl(), rect.br(), new Scalar(0, 0, 255), 2);

        }
        establishSets.add(contoursList);

        return drawingMat;
    }



}