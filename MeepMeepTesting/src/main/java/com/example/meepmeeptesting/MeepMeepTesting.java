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
        Pose2d StartPositionLeft = new Pose2d(-35,-60,Math.toRadians(270));
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
                                .back(10)
                                .splineToLinearHeading(new Pose2d(10,-30,Math.toRadians(315)), Math.toRadians(0))
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(25,-33,Math.toRadians(0)),Math.toRadians(0))
                                .lineTo(new Vector2d(40,-33))
                                .splineToSplineHeading(new Pose2d(50,-30,Math.toRadians(45)),Math.toRadians(45))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(40,-33,Math.toRadians(0)),Math.toRadians(180))
                                .lineTo(new Vector2d(25,-33))
                                .splineToSplineHeading(new Pose2d(10,-30,Math.toRadians(315)),Math.toRadians(135))
                                .build()
                );
        RoadRunnerBotEntity Autonomia4 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 50, 5.69, 5.69, 7)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(StartPositionRight)
                                .lineToLinearHeading(new Pose2d(35,-6,Math.toRadians(345)))
                                .splineToSplineHeading(new Pose2d(10,-13,Math.toRadians(180)),Math.toRadians(180))
                                .lineTo(new Vector2d(-25,-13))
                                .splineToSplineHeading(new Pose2d(-31,-11,Math.toRadians(192)),Math.toRadians(192))
                                .build()
                );
        RoadRunnerBotEntity Autonomia5 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 50, 5.69, 5.69, 7)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(StartPositionRight)
                                .lineToLinearHeading(new Pose2d(35,-23,Math.toRadians(20)))
                                .lineTo(new  Vector2d(35, -21))
                                .splineToSplineHeading(new Pose2d(10,-10,Math.toRadians(180)),Math.toRadians(180))
                                .lineTo(new Vector2d(-20,-10))
                                .splineToSplineHeading(new Pose2d(-35,-23,Math.toRadians(160)),Math.toRadians(270))
                                .build()
                );
        RoadRunnerBotEntity Autonomia7 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 50, 5.69, 5.69, 7)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(StartPositionLeft)
                                .lineToLinearHeading(new Pose2d(-34.5,-23,Math.toRadians(320)))
                                .build()
                );
        RoadRunnerBotEntity LeftAutonomySouthHigh5_1 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 50, 5.69, 5.69, 7)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(StartPositionLeft)
                                .lineToLinearHeading(new Pose2d(-34.5,-14,Math.toRadians(217)))
                                .turn(Math.toRadians(-37))
                                .lineToLinearHeading(new Pose2d(-10,-14,Math.toRadians(150)))
                                .lineToLinearHeading(new Pose2d(-34.5,-14,Math.toRadians(180)))
                                .build()
                );
        RoadRunnerBotEntity RightAutonomySouthHigh5_1 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 50, 5.69, 5.69, 7)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(StartPositionRight)
                                .lineToLinearHeading(new Pose2d(33,-10,Math.toRadians(315)))
                                .turn(Math.toRadians(45))
                                .setReversed(true)
                                .lineTo(new Vector2d(25,-10))
                                .splineToSplineHeading(new Pose2d(5,-15,Math.toRadians(45)),Math.toRadians(225))
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(25,-10,Math.toRadians(0)),Math.toRadians(0))
                                .lineTo(new Vector2d(33,-10))
                                .setReversed(true)
                                .lineTo(new Vector2d(25,-9))
                                .splineToSplineHeading(new Pose2d(5,-14,Math.toRadians(45)),Math.toRadians(225))
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(25,-9,Math.toRadians(0)),Math.toRadians(0))
                                .lineTo(new Vector2d(33,-9))
                                .setReversed(true)
                                .lineTo(new Vector2d(25,-8.5))
                                .splineToSplineHeading(new Pose2d(5,-13.5,Math.toRadians(45)),Math.toRadians(225))
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(25,-8.5,Math.toRadians(0)),Math.toRadians(0))
                                .lineTo(new Vector2d(33,-8.5))
                                .setReversed(true)
                                .lineTo(new Vector2d(25,-8))
                                .splineToSplineHeading(new Pose2d(5,-13,Math.toRadians(45)),Math.toRadians(225))
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(25,-8,Math.toRadians(0)),Math.toRadians(0))
                                .lineTo(new Vector2d(33,-8))
                                .setReversed(true)
                                .lineTo(new Vector2d(25,-7.5))
                                .splineToSplineHeading(new Pose2d(5,-12.5,Math.toRadians(45)),Math.toRadians(225))
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(25,-7.5,Math.toRadians(0)),Math.toRadians(0))
                                .lineTo(new Vector2d(33,-7.5))
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(Autonomia5)
                .start();
    }
}