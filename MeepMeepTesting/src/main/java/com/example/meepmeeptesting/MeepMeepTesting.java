package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;



public class MeepMeepTesting {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);

        // new starting pose
        Pose2d initialPose = new Pose2d(63, 35, Math.toRadians(90));


        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        bot.setPose(initialPose);

        // Trajectory Variables
        TrajectoryActionBuilder firstLaunch = bot.getDrive().actionBuilder(initialPose).strafeToLinearHeading(new Vector2d(55,26), Math.toRadians(90));
        TrajectoryActionBuilder firstPickup = firstLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(36, 28)).strafeToConstantHeading(new Vector2d(36,48), new TranslationalVelConstraint(6));
        TrajectoryActionBuilder secondLaunch = firstPickup.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(55,26));
        TrajectoryActionBuilder secondPickup = secondLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(12, 28)).strafeToConstantHeading(new Vector2d(12,48) , new TranslationalVelConstraint(6));
        TrajectoryActionBuilder thirdLaunch = secondPickup.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(55,26));
        TrajectoryActionBuilder thirdPickup = thirdLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-11,28)).strafeToConstantHeading(new Vector2d(-11,48), new TranslationalVelConstraint(6));
        TrajectoryActionBuilder fourthLaunch = thirdPickup.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(55,26));
        TrajectoryActionBuilder park = fourthLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-7,34));

        // run in order
        bot.runAction(
                new SequentialAction(
                        firstLaunch.build(),
                        firstPickup.build(),
                        secondLaunch.build(),
                        secondPickup.build(),
                        thirdLaunch.build(),
                        thirdPickup.build(),
                        fourthLaunch.build(),
                        park.build()
                )
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();

    }
}