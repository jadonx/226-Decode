package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class ScrimBlueSideCloseAutoMeepMeep {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // new starting pose
        Pose2d startPose = new Pose2d(-48, -48, Math.toRadians(225));

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        // so the bot spawns here
        bot.setPose(startPose);

        // Trajectory Variables
        TrajectoryActionBuilder moveToShootArtifacts = bot.getDrive().actionBuilder(startPose)
                .lineToY(-20);

        // run in order
        bot.runAction(
                new SequentialAction(
                        moveToShootArtifacts.build()
                )
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();

    }
}
