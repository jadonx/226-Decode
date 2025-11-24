package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;



public class Path2 {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);

        // new starting pose
        Pose2d startPose = new Pose2d(-48, -48, Math.toRadians(54));

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        // so the bot spawns here
        bot.setPose(startPose);

        // Trajectory Variables
        TrajectoryActionBuilder shiftForward = bot.getDrive().actionBuilder(startPose)
                .setTangent(Math.toRadians(90))
                .lineToYLinearHeading(-44, Math.toRadians(90))
                ;
        TrajectoryActionBuilder shootZone = shiftForward.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(270)), Math.toRadians(0.00))
                .waitSeconds(2)
                ;
        TrajectoryActionBuilder collectSample1 = shootZone.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-11.5, -28, Math.toRadians(270)), Math.toRadians(0.00))
                ;

        TrajectoryActionBuilder collectSample10 = collectSample1.endTrajectory().fresh()
                .setReversed(false)
                //.splineToLinearHeading(new Pose2d(-11, -47, Math.toRadians(270)), Math.toRadians(0.00))
                .lineToY(-35.5)
                ;
        TrajectoryActionBuilder collectSample11 = collectSample10.endTrajectory().fresh()
                .setReversed(false)
                //.splineToLinearHeading(new Pose2d(-11, -47, Math.toRadians(270)), Math.toRadians(0.00))
                .lineToY(-40.5)
                ;
        TrajectoryActionBuilder collectSample12 = collectSample11.endTrajectory().fresh()
                .setReversed(false)
                //.splineToLinearHeading(new Pose2d(-11, -47, Math.toRadians(270)), Math.toRadians(0.00))
                .lineToY(-45.5)
                ;
        TrajectoryActionBuilder pushLever = collectSample12.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(-6, -48, Math.toRadians(270)), Math.toRadians(0.00))
                ;
        TrajectoryActionBuilder pushLever2 =  pushLever.endTrajectory().fresh()
                .setReversed(false)
                //.splineToLinearHeading(new Pose2d(-11, -47, Math.toRadians(270)), Math.toRadians(0.00))
                .lineToY(-52)
                .waitSeconds(3)
                ;


        TrajectoryActionBuilder shootZone2 = pushLever2.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(270)), Math.toRadians(0.00))
                .waitSeconds(2)
                ;
        TrajectoryActionBuilder collectSample2 = shootZone2.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(11.5, -28, Math.toRadians(270)), Math.toRadians(0.00));
        ;

        TrajectoryActionBuilder collectSample20 = collectSample2.endTrajectory().fresh()
                .setReversed(false)
                //.splineToLinearHeading(new Pose2d(-11, -47, Math.toRadians(270)), Math.toRadians(0.00))
                .lineToY(-35.5)
                ;
        TrajectoryActionBuilder collectSample21 = collectSample20.endTrajectory().fresh()
                .setReversed(false)
                //.splineToLinearHeading(new Pose2d(-11, -47, Math.toRadians(270)), Math.toRadians(0.00))
                .lineToY(-40.5)
                ;
        TrajectoryActionBuilder collectSample22 = collectSample21.endTrajectory().fresh()
                .setReversed(false)
                //.splineToLinearHeading(new Pose2d(-11, -47, Math.toRadians(270)), Math.toRadians(0.00))
                .lineToY(-45.5)
                ;
        TrajectoryActionBuilder shootZone3 = collectSample22.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(270)), Math.toRadians(0.00))
                .waitSeconds(2)
                ;
        TrajectoryActionBuilder collectSample3 = shootZone3.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(36, -28, Math.toRadians(270)), Math.toRadians(0.00));
        ;
        TrajectoryActionBuilder collectSample30 = collectSample3.endTrajectory().fresh()
                .setReversed(false)
                //.splineToLinearHeading(new Pose2d(-11, -47, Math.toRadians(270)), Math.toRadians(0.00))
                .lineToY(-35.5)
                ;
        TrajectoryActionBuilder collectSample31 = collectSample30.endTrajectory().fresh()
                .setReversed(false)
                //.splineToLinearHeading(new Pose2d(-11, -47, Math.toRadians(270)), Math.toRadians(0.00))
                .lineToY(-40.5)
                ;
        TrajectoryActionBuilder collectSample32 = collectSample31.endTrajectory().fresh()
                .setReversed(false)
                //.splineToLinearHeading(new Pose2d(-11, -47, Math.toRadians(270)), Math.toRadians(0.00))
                .lineToY(-45.5)
                ;

        TrajectoryActionBuilder shootZone4 = collectSample32.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(270)), Math.toRadians(0.00))
                .waitSeconds(2)
                ;

        // run in order
        bot.runAction(
                new SequentialAction(

                        shiftForward.build(),
                        shootZone.build(),
                        collectSample1.build(),
                        collectSample10.build(),
                        collectSample11.build(),
                        collectSample12.build(),
                        pushLever.build(),
                        pushLever2.build(),
                        shootZone2.build(),
                        collectSample2.build(),
                        collectSample20.build(),
                        collectSample21.build(),
                        collectSample22.build(),
                        shootZone3.build(),
                        collectSample3.build(),
                        collectSample30.build(),
                        collectSample31.build(),
                        collectSample32.build(),

                        shootZone4.build()

                )
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();

    }
}










/*
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
*/
