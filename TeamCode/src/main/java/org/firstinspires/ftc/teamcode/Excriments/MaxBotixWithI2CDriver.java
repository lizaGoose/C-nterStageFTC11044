package org.firstinspires.ftc.teamcode.Excriments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@TeleOp
public class MaxBotixWithI2CDriver extends LinearOpMode {

    private MB1242 maxBot;
    private DistanceUnit unit = DistanceUnit.INCH;
    @Override
    public void runOpMode() throws InterruptedException {

        maxBot = hardwareMap.get(MB1242.class, "tempSensor");

        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Distance", unit.fromCm(maxBot.getDistance(DistanceUnit.CM)));
            telemetry.update();
        }
    }
}
