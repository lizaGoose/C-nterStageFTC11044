package org.firstinspires.ftc.teamcode.Excriments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@TeleOp
public class DinstanceSensor extends LinearOpMode {

    DistanceSensor sensorDistance;
    @Override
    public void runOpMode() throws InterruptedException {

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
            telemetry.update();
        }
    }
}
