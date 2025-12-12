package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;



public class MMBlueClose {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);

        // new starting pose
        Pose2d startPose = new Pose2d(-60, -20, Math.toRadians(270));
        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep).setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15).build();
        // so the bot spawns here
        bot.setPose(startPose);

        TrajectoryActionBuilder firstLaunch = bot.getDrive().actionBuilder(startPose).strafeToLinearHeading(new Vector2d(-10,-40), Math.toRadians(270));
        TrajectoryActionBuilder firstPickup = firstLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-10,-40)).strafeToConstantHeading(new Vector2d(-10,-55), new TranslationalVelConstraint(7));
        TrajectoryActionBuilder secondLaunch = firstPickup.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-7,-35));
        TrajectoryActionBuilder secondPickup = secondLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(15, -40)).strafeToConstantHeading(new Vector2d(15,-55) , new TranslationalVelConstraint(7));
        TrajectoryActionBuilder thirdLaunch = secondPickup.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-7,-35));
        TrajectoryActionBuilder thirdPickup = thirdLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(39, -40)).strafeToConstantHeading(new Vector2d(39,-55), new TranslationalVelConstraint(7));
        TrajectoryActionBuilder fourthLaunch = thirdPickup.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-7,-35));
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