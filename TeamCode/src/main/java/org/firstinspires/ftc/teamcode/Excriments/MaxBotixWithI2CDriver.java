package org.firstinspires.ftc.teamcode.Excriments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@TeleOp
public class MaxBotixWithI2CDriver extends LinearOpMode {

    private MB1242 maxBot;
    @Override
    public void runOpMode() throws InterruptedException {
        maxBot = hardwareMap.get(MB1242.class, "tempSensor");
        // get a reference to the distance sensor that shares the same name.
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Distance", maxBot.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}
