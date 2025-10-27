package org.firstinspires.ftc.teamcode.helpers.camera.exCam;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class colorDetectionPipeline implements VisionProcessor {

    private volatile Rect detectedRect = new Rect();

    private Mat workingMat = new Mat();
    private Mat hierarchy = new Mat();

    private Paint paint = new Paint();

    public Rect getDetectedRect() {
        return detectedRect;
    }

    public static double score = 0;
    public static double height = 0;
    public static double width = 0;


    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        paint.setColor(Color.GREEN);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(10);
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {

        Imgproc.cvtColor(input, workingMat, Imgproc.COLOR_RGB2HSV);

        Core.inRange(workingMat, new Scalar(150, 60, 60), new Scalar(190, 255, 255), workingMat);
        List<MatOfPoint> contoursList = new ArrayList<>();
        Imgproc.findContours(workingMat, contoursList, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        Rect bestRect = new Rect();
        double lowestScore = Double.MAX_VALUE;

        for (MatOfPoint contour : contoursList) {
            double currentScore = calculateScore(contour);

            Rect rect = Imgproc.boundingRect(contour);

            if (currentScore < lowestScore) {
                lowestScore = currentScore;
                bestRect = rect;
            }

        }

        detectedRect = bestRect;

        score = detectedRect.height * detectedRect.width;
        height = detectedRect.height;
        width = detectedRect.width;

        return detectedRect;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

        Rect rect = (Rect) userContext;

        if (rect != null && rect.width > 0 && rect.height > 0) {

            float left = (float) rect.x * scaleBmpPxToCanvasPx;
            float top = (float) rect.y * scaleBmpPxToCanvasPx;
            float right = (float) (rect.x + rect.width) * scaleBmpPxToCanvasPx;
            float bottom = (float) (rect.y + rect.height) * scaleBmpPxToCanvasPx;

            canvas.drawRect(left, top, right, bottom, paint);

        }
    }


    private double calculateScore(Mat input) {
        if(!(input instanceof MatOfPoint))
            return Double.MAX_VALUE;
        return -Imgproc.contourArea(input);
    }
}