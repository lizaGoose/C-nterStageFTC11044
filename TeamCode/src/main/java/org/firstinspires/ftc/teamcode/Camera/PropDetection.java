package org.firstinspires.ftc.teamcode.Camera;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
public class PropDetection extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();

    public enum Location {
        LEFT,
        CENTER,
        RIGHT,
        NOT_FOUND
    }
    private Location location = Location.CENTER;

    static final Rect LEFT_ROI = new Rect(
            new Point(580, 210),
            new Point(700, 320));
    static final Rect CENTER_ROI = new Rect(
            new Point(270, 240),
            new Point(500, 170));
    static final Rect RIGHT_ROI = new Rect(
            new Point(200, 210),
            new Point(120, 300));

    static double PERCENT_COLOR_THRESHOLD = 0.4;

    public PropDetection(Telemetry t) { telemetry = t; }
    @Override    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Scalar lowHSV = new Scalar(100, 150, 0);
        Scalar highHSV = new Scalar(140, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat center = mat.submat(CENTER_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double centerValue = Core.sumElems(left).val[0] / CENTER_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        left.release();
        center.release();
        right.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Center raw value", (int) Core.sumElems(center).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);

        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Center percentage", Math.round(centerValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");

        boolean propLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean propCenter = centerValue > PERCENT_COLOR_THRESHOLD;
        boolean propRight = rightValue > PERCENT_COLOR_THRESHOLD;

        if (Math.round(rightValue * 100) > 20) {
            location = Location.RIGHT;
            telemetry.addData("Prop Location", "left");
        }

        else if(Math.round(leftValue * 100) >= 15) {
            location = Location.LEFT;
            telemetry.addData("Prop Location", "right");
        }
        else{
            location = Location.CENTER;
            telemetry.addData("Prop Location", "center");
        }

        telemetry.update();
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorNoProp = new Scalar(255, 0, 0);
        Scalar colorProp = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT? colorProp:colorNoProp);
        Imgproc.rectangle(mat, CENTER_ROI, location == Location.CENTER? colorProp:colorNoProp);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? colorProp:colorNoProp);

        return mat;    }
    public Location getLocation() {
        return location;    }}
