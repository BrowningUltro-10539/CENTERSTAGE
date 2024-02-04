package org.firstinspires.ftc.teamcode.Vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;



public class BluePipeline implements VisionProcessor {

    volatile MarkerPosition markerPosition = MarkerPosition.CENTER;
    static final Scalar BLUE_LOWER = new Scalar(100, 150, 150);
    static final Scalar BLUE_UPPER = new Scalar(140, 255, 255);

    /**
     * FOR RED:
     *
     * Remove the Scalar bounds for blue (above)
     * static final Scalar RED_LOWER = new Scalar(0, 70, 50);
     * static final Scalar RED_UPPER = new Scalar(10, 255, 255);
     *
     */

    @Override
    public void init(int width, int height, CameraCalibration calibration) {}

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Mat mat = new Mat();

        /**
         * CHANGE THE BOUNDS HERE
         */
        Core.inRange(frame, BLUE_LOWER, BLUE_UPPER, mat);

        markerPosition = analyzeRegions(mat);

        return mat;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {}


    private MarkerPosition analyzeRegions(Mat mat) {
        Rect leftRegion = new Rect(0, 0, mat.cols() / 3, mat.rows());
        Rect centerRegion = new Rect(mat.cols() / 3, 0, mat.cols() / 3, mat.rows());
        Rect rightRegion = new Rect(2 * mat.cols() / 3, 0, mat.cols() / 3, mat.rows());


        double leftSum = Core.sumElems(mat.submat(leftRegion)).val[0];
        double centerSum = Core.sumElems(mat.submat(centerRegion)).val[0];
        double rightSum = Core.sumElems(mat.submat(rightRegion)).val[0];

        if (leftSum > centerSum && leftSum > rightSum) return MarkerPosition.LEFT;
        else if (centerSum > leftSum && centerSum > rightSum) return MarkerPosition.CENTER;
        else if (rightSum > leftSum && rightSum > centerSum) return MarkerPosition.RIGHT;
        else return MarkerPosition.CENTER;
    }

    public MarkerPosition getMarkerPosition() {
        return markerPosition;
    }
}
