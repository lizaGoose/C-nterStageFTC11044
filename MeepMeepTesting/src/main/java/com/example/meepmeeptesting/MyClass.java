package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MyClass {
    public  static double xSplineOutOfWallRight = -43,ySplineOutOfWallRight =  -56,
            xSpikeRight = -55, ySpikeRight =  -33,
            xSteakRunning0Right = -55, ySteakRunning0Right = -25,
            xSteakRunning1Right = -60, ySteakRunning1Right = -11,
            xBackdropRunning1Right = 38 ,yBackdropRunning1Right = -10,
            xBackdropRunning2Right =  50, yBackdropRunning2Right = -28,
            xSteakRunning2Right=38, ySteakRunning2Right=-10,
            xSteakRunning3Right = -30,ySteakRunning3Right = -5,
            xBackdropRunning3Right = 38, yBackdropRunning3Right = -10,
            xBackdropRunning4Right = 50, yBackdropRunning4Right = -33,
            xParkingRight = 45, yParkingRight = -10;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)


                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(38.945723632307676, 38.156578905846143, Math.toRadians(180), Math.toRadians(180), 6.93)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-38, -61.5, 0))
                                .splineToLinearHeading(new Pose2d(xSplineOutOfWallRight, ySplineOutOfWallRight, Math.toRadians(0)), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(xSpikeRight, ySpikeRight, Math.toRadians(0)), Math.toRadians(0))
                                .waitSeconds(0.3)
                                .splineToLinearHeading(new Pose2d(xSteakRunning0Right, ySteakRunning0Right, Math.toRadians(0)), Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(xSteakRunning1Right, ySteakRunning1Right, Math.toRadians(0)), Math.toRadians(0))
                                .waitSeconds(1)
                                .splineToLinearHeading(new Pose2d(xBackdropRunning1Right, yBackdropRunning1Right, Math.toRadians(0)), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(xBackdropRunning2Right, yBackdropRunning2Right, Math.toRadians(0)), Math.toRadians(0))
                                .waitSeconds(0.5)
                                .splineToLinearHeading(new Pose2d(xSteakRunning2Right, ySteakRunning2Right, Math.toRadians(0)), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(xSteakRunning3Right, ySteakRunning3Right, Math.toRadians(10)), Math.toRadians(180))
                                .waitSeconds(1)
                                .splineToLinearHeading(new Pose2d(xBackdropRunning3Right, yBackdropRunning3Right, Math.toRadians(0)), Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(xBackdropRunning4Right, yBackdropRunning4Right, Math.toRadians(0)), Math.toRadians(0))
                                .waitSeconds(0.5)
                                .splineToLinearHeading(new Pose2d(xParkingRight, yParkingRight, Math.toRadians(0)), Math.toRadians(0))
                                .build()
                                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}