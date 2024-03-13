package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MyClass {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)


                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(38.945723632307676, 38.156578905846143, Math.toRadians(180), Math.toRadians(180), 6.93)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-38, 61.5, 0))
                                .splineToLinearHeading(new Pose2d(-5, 5, Math.toRadians(0)), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(-10, 35, Math.toRadians(0)), Math.toRadians(0))
                                .waitSeconds(0.3)
                                .splineToLinearHeading(new Pose2d(-22.5, 29.6, 0), Math.toRadians(90))
                                .lineToLinearHeading(new Pose2d(-22.6, 50.5, 0))
                                .waitSeconds(0.7)

                                .splineToConstantHeading(new Vector2d(60, 52), 0)
                                .splineToLinearHeading(new Pose2d(75, 36, 0), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(95.7, 24.5, 0), Math.toRadians(0))
                                .waitSeconds(5)
                                .splineToLinearHeading(new Pose2d(77.7, 26.55, 0), Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(70, 52, 0), Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(-21, 50, 0), Math.toRadians(-90))
                                .waitSeconds(0.5)
                                .splineToConstantHeading(new Vector2d(60, 53), 0)
                                .splineToLinearHeading(new Pose2d(75, 36, 0), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(89.7, 28.7, 0), Math.toRadians(0))
                                .waitSeconds(0.4)
                                .splineToLinearHeading(new Pose2d(89.7, 28.77, 0), Math.toRadians(180))

                                .splineToLinearHeading(new Pose2d(70, 54, 0), Math.toRadians(180))
                                .build()
                                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}