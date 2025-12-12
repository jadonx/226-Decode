package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;



public class MMRedClose {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);

        // new starting pose
        Pose2d startPose = new Pose2d(-60, 20, Math.toRadians(90));

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        // so the bot spawns here
        bot.setPose(startPose);

        // Trajectory Variables
        TrajectoryActionBuilder firstLaunch = bot.getDrive().actionBuilder(startPose)
                .strafeToLinearHeading(new Vector2d(-11.5,20), Math.toRadians(90))
                .waitSeconds(3);

        TrajectoryActionBuilder firstPickup = firstLaunch.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-11.5, 30))
                .strafeToConstantHeading(new Vector2d(-11.5,50), new TranslationalVelConstraint(10))
                ;


        TrajectoryActionBuilder secondLaunch = firstPickup.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-11.5,20))
                .waitSeconds(3);

        TrajectoryActionBuilder secondPickup = secondLaunch.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(13, 30))
                .strafeToConstantHeading(new Vector2d(13,50) , new TranslationalVelConstraint(10));


        TrajectoryActionBuilder thirdLaunch = secondPickup.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-11.5,20))
                .waitSeconds(3);

        TrajectoryActionBuilder thirdPickup = thirdLaunch.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(37, 30))
                .strafeToConstantHeading(new Vector2d(37,50), new TranslationalVelConstraint(10))
                ;

        TrajectoryActionBuilder fourthLaunch = thirdPickup.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-11.5,20))
                .waitSeconds(3);

        // run in order
        bot.runAction(
                new SequentialAction(
                        firstLaunch.build(),
                        firstPickup.build(),
                        secondLaunch.build(),
                        secondPickup.build(),
                        thirdLaunch.build(),
                        thirdPickup.build(),
                        fourthLaunch.build()
                )
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();

    }
}