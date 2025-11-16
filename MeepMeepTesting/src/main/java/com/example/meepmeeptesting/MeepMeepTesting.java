package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d initialPose = new Pose2d(-50, -50, Math.toRadians(229));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(120, 120, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)
                .strafeTo(new Vector2d(-12, -10))
                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(-12, -20), Math.toRadians(270))
                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(-12, -50), Math.toRadians(270))
                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(-12, -10), Math.toRadians(229))
                .waitSeconds(.2)
                .strafeToLinearHeading(new Vector2d(12, -20), Math.toRadians(270))
                .waitSeconds(.2)
                .strafeToLinearHeading(new Vector2d(12, -50), Math.toRadians(270))
                .waitSeconds(.2)
                .strafeToLinearHeading(new Vector2d(-12, -10), Math.toRadians(229))
                .waitSeconds(.2)
                .strafeToLinearHeading(new Vector2d(35.5, -20), Math.toRadians(270))
                .waitSeconds(.2)
                .strafeToLinearHeading(new Vector2d(35.5, -50), Math.toRadians(270))
                .waitSeconds(.2)
                .strafeToLinearHeading(new Vector2d(-12, -10), Math.toRadians(229))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}