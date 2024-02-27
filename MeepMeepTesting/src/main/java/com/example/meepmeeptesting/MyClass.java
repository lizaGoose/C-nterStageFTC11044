package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MyClass {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)


                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(38.945723632307676, 38.156578905846143, Math.toRadians(180), Math.toRadians(180), 6.93)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-34, 56, 0))
                                .splineToLinearHeading(new Pose2d(-5, -5, Math.toRadians(0)), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(-10, -38, Math.toRadians(0)), Math.toRadians(0))
                                .waitSeconds(0.3)

                                .splineToLinearHeading(new Pose2d(-23.5, -52, 0), Math.toRadians(80))
                                .waitSeconds(0.9)

                                .waitSeconds(0.2)


                                //.splineToConstantHeading(new Vector2d(48, -50), 0)
                                .splineToLinearHeading(new Pose2d(88.7, -23.5, 0), Math.toRadians(60))
                                .waitSeconds(0.5)
                                .splineToLinearHeading(new Pose2d(70, -49.5, 0), Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(0, -49.5, 0), Math.toRadians(180))
                                .waitSeconds(0.6)



                                //splineToConstantHeading(new Vector2d(48, -50), 0)

                                .splineToLinearHeading(new Pose2d(88, -28.5, 0), Math.toRadians(60))
                                .waitSeconds(0.3)

                                .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}