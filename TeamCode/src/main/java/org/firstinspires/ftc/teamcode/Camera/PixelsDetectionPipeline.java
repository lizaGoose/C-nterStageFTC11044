package org.firstinspires.ftc.teamcode.Camera;

import android.graphics.ColorSpace;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class PixelsDetectionPipeline extends OpenCvPipeline{

    Telemetry telemetry;
    public PixelsDetectionPipeline (Telemetry t){telemetry = t;}
   public boolean first_place;
    Mat mat = new Mat();

    Mat output = new Mat();

    LinearOpMode opMode;

    public enum Status{
        SCORED,
        FREE,
        NOT_FOUND
    }
    public Status status = Status.NOT_FOUND;
    static final Rect FIRST = new Rect(
            new Point(270, 275),
            new Point(235, 250)
    );
    static double THRESHOLD = 0.4;
    @Override
    public Mat processFrame(Mat input) {
       // Rect first_rect = new Rect(180,120,50,50);

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb);
        Scalar low_black = new Scalar(0, 200, 0);
        Scalar high_black = new Scalar(0, 255, 0);

        input.copyTo(output);
        Core.inRange(mat, low_black, high_black, mat);
        Mat first = input.submat(FIRST);

       // double first_value = Core.mean(first).val[0];

        double first_value = Core.sumElems(first).val[0] / FIRST.area() / 255;


        first.release();

        telemetry.addData("Left percentage", first_value);
        telemetry.update();
        first_place = first_value > THRESHOLD;
        Scalar colorScalar = new Scalar(0.0, 255.0, 0.0);


        switch (status) {
            case FREE:
                telemetry.speak("free");
                break;
            case SCORED:
                telemetry.speak("scored");
                break;
            case NOT_FOUND:
                telemetry.speak("not found");
                break;
        }

        Imgproc.rectangle(output, FIRST,colorScalar, 2);



        return output;
    }
}


