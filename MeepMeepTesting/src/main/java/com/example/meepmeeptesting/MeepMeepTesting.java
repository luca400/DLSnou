package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d StartPositionRight = new Pose2d(35,-60,Math.toRadians(270));
        RoadRunnerBotEntity Autonomia1 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 50, 5.69, 5.69, 7)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(StartPositionRight)
                                .lineToLinearHeading(new Pose2d(33,3,Math.toRadians(335)))
                                .build()
                );
        RoadRunnerBotEntity Autonomia2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 50, 5.69, 5.69, 7)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(StartPositionRight)
                                .lineToLinearHeading(new Pose2d(33,-23,Math.toRadians(20)))
                                .build()
                );
        RoadRunnerBotEntity Autonomia3 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 50, 5.69, 5.69, 7)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(StartPositionRight)
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(10,-30,Math.toRadians(315)), Math.toRadians(0))
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(25,-33,Math.toRadians(0)),Math.toRadians(0))
                                .lineTo(new Vector2d(40,-33))
                                .splineToSplineHeading(new Pose2d(50,-30,Math.toRadians(45)),Math.toRadians(45))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(40,-33,Math.toRadians(0)),Math.toRadians(0))
                                .lineTo(new Vector2d(25,-33))
                                .splineToSplineHeading(new Pose2d(10,-30,Math.toRadians(315)),Math.toRadians(315))
                                .build()
                );
        RoadRunnerBotEntity Autonomia4 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 50, 5.69, 5.69, 7)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(StartPositionRight)
                                .lineToLinearHeading(new Pose2d(35,-6,Math.toRadians(345)))
                                .splineToSplineHeading(new Pose2d(10,-13,Math.toRadians(180)),Math.toRadians(180))
                                .lineTo(new Vector2d(-25,-16))
                                .lineToLinearHeading(new Pose2d(-31,-11,Math.toRadians(192)))
                                .build()
                );
        RoadRunnerBotEntity Autonomia5 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 50, 5.69, 5.69, 7)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(StartPositionRight)
                                .lineToLinearHeading(new Pose2d(35,-23,Math.toRadians(20)))
                                .splineToSplineHeading(new Pose2d(10,-10,Math.toRadians(90)),Math.toRadians(180))
                                .lineTo(new Vector2d(-20,-10))
                                .splineToSplineHeading(new Pose2d(-35,-23,Math.toRadians(160)),Math.toRadians(20))
                                .build()
                );
        RoadRunnerBotEntity Autonomia7 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 50, 5.69, 5.69, 7)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(StartPositionRight)
                                .turn(Math.toRadians(45))
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(Autonomia4)
                .start();
    }
}