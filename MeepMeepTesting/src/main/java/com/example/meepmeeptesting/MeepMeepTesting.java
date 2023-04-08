package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        double INTER_SPLINE_X = 13, INTER_SPLINE_Y = -50;
        double x_CYCLING_POSITION = 13, y_CYCLING_POSITION = -61;
        double x_COLLECT_POSITION = 17, y_COLLECT_POSITION = -18, Angle_COLLECT_POSITION = 6;
        double x_PLACE_SOUTH_HIGH = 14, y_PLACE_SOUTH_HIGH = -20, Angle_PLACE_SOUTH_HIGH = 16;
        double x_COLLECT_POSITION_LEFT = -17, y_COLLECT_POSITION_LEFT = -13, Angle_COLLECT_POSITION_LEFT = 180;
        double x_PLACE_SOUTH_HIGH_LEFT = -14, y_PLACE_SOUTH_HIGH_LEFT = -18.5, Angle_PLACE_SOUTH_HIGH_LEFT = 166;
        double x_PARK1 = -11.5, y_PARK1 = -14, Angle_PARK1 = 180;
        double x_PARK2 = -33, y_PARK2 = -14.5, Angle_PARK2 = 180;
        double x_PARK3 = -56, y_PARK3 = -14, Angle_PARK3 = 180;
        Pose2d StartPositionRight = new Pose2d(35,-63,Math.toRadians(270));
        Pose2d PLACE_SOUTH_HIGH = new Pose2d(x_PLACE_SOUTH_HIGH,y_PLACE_SOUTH_HIGH,Math.toRadians(Angle_PLACE_SOUTH_HIGH));
        Pose2d PLACE_SOUTH_HIGH_LEFT = new Pose2d(x_PLACE_SOUTH_HIGH_LEFT,y_PLACE_SOUTH_HIGH_LEFT,Math.toRadians(Angle_PLACE_SOUTH_HIGH_LEFT));
        Pose2d COLLECT_POSITION_5 = new Pose2d(x_COLLECT_POSITION,y_COLLECT_POSITION,Math.toRadians(Angle_COLLECT_POSITION));
        Pose2d COLLECT_POSITION_4 = new Pose2d(x_COLLECT_POSITION,y_COLLECT_POSITION,Math.toRadians(Angle_COLLECT_POSITION));
        Pose2d COLLECT_POSITION_3 = new Pose2d(x_COLLECT_POSITION,y_COLLECT_POSITION,Math.toRadians(Angle_COLLECT_POSITION+1));
        Pose2d COLLECT_POSITION_2 = new Pose2d(x_COLLECT_POSITION,y_COLLECT_POSITION,Math.toRadians(Angle_COLLECT_POSITION+1));
        Pose2d COLLECT_POSITION_1 = new Pose2d(x_COLLECT_POSITION,y_COLLECT_POSITION,Math.toRadians(Angle_COLLECT_POSITION+1.5));



        Pose2d COLLECT_POSITION_6 = new Pose2d(x_COLLECT_POSITION_LEFT,y_COLLECT_POSITION_LEFT,Math.toRadians(Angle_COLLECT_POSITION_LEFT-0.8));
        Pose2d COLLECT_POSITION_7 = new Pose2d(x_COLLECT_POSITION_LEFT,y_COLLECT_POSITION_LEFT,Math.toRadians(Angle_COLLECT_POSITION_LEFT-2));
        Pose2d COLLECT_POSITION_8 = new Pose2d(x_COLLECT_POSITION_LEFT,y_COLLECT_POSITION_LEFT,Math.toRadians(Angle_COLLECT_POSITION_LEFT-3));
        Pose2d COLLECT_POSITION_9 = new Pose2d(x_COLLECT_POSITION_LEFT,y_COLLECT_POSITION_LEFT,Math.toRadians(Angle_COLLECT_POSITION_LEFT-3));
        Pose2d COLLECT_POSITION_10 = new Pose2d(x_COLLECT_POSITION_LEFT,y_COLLECT_POSITION_LEFT,Math.toRadians(Angle_COLLECT_POSITION_LEFT-3));
        Pose2d StartPositionLeft = new Pose2d(-35,-63,Math.toRadians(270));
        RoadRunnerBotEntity Autonomia1 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                // .setConstraints(60, 50, 5.69, 5.69, 7)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(StartPositionRight)
                                .lineTo(new Vector2d(31,-62))
                                .splineToConstantHeading(new Vector2d(INTER_SPLINE_X,INTER_SPLINE_Y),Math.toRadians(90))
                                .splineToSplineHeading(PLACE_SOUTH_HIGH,Math.toRadians(90))
                                .lineToLinearHeading(COLLECT_POSITION_5)
                                .lineToLinearHeading(PLACE_SOUTH_HIGH)
                                .lineToLinearHeading(COLLECT_POSITION_4)
                                .lineToLinearHeading(PLACE_SOUTH_HIGH)
                                .lineToLinearHeading(COLLECT_POSITION_3)
                                .lineToLinearHeading(PLACE_SOUTH_HIGH)
                                .lineToLinearHeading(COLLECT_POSITION_2)
                                .lineToLinearHeading(PLACE_SOUTH_HIGH)
                                .lineToLinearHeading(COLLECT_POSITION_1)
                                .lineToLinearHeading(PLACE_SOUTH_HIGH)
                                .lineToLinearHeading(COLLECT_POSITION_6)
                                .lineToLinearHeading(PLACE_SOUTH_HIGH_LEFT)
                                .lineToLinearHeading(COLLECT_POSITION_7)
                                .lineToLinearHeading(PLACE_SOUTH_HIGH_LEFT)
                                .lineToLinearHeading(COLLECT_POSITION_8)
                                .lineToLinearHeading(PLACE_SOUTH_HIGH_LEFT)
                                .lineToLinearHeading(COLLECT_POSITION_9)
                                .lineToLinearHeading(PLACE_SOUTH_HIGH_LEFT)
                                .lineToLinearHeading(COLLECT_POSITION_10)
                                .lineToLinearHeading(new Pose2d(x_PARK1,y_PARK1,Math.toRadians(Angle_PARK1)))
                                .build()
                );

        RoadRunnerBotEntity Autonomia2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                // .setConstraints(60, 50, 5.69, 5.69, 7)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(StartPositionRight)
                                .lineToLinearHeading(new Pose2d(36.5,-5.26,Math.toRadians(347.5)))
                                .lineTo(new  Vector2d(34.5, -6.26))
                                .splineToSplineHeading(new Pose2d(-15,-12.6,Math.toRadians(180)),Math.toRadians(180))
                                .lineTo(new Vector2d(-30,-12.6))
                                .lineToLinearHeading(new Pose2d(-35,-4.8,Math.toRadians(194)))
                                .lineTo(new Vector2d(-50,-23))

                                .build()
                );
        RoadRunnerBotEntity Autonomia3 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 50, 5.69, 5.69, 7)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(StartPositionRight)
                                .lineToLinearHeading(new Pose2d(37.5,-12,Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(36.5,-19.5,Math.toRadians(12)))
                                .lineToLinearHeading(new Pose2d(35.5,-30,Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(10,-25,Math.toRadians(270)))
                                .build()
                );
        RoadRunnerBotEntity Autonomia4 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 50, 5.69, 5.69, 7)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(StartPositionRight)
                                .lineToLinearHeading(new Pose2d(37.5,-12,Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(17,-18,Math.toRadians(7.2)))
                                .lineToLinearHeading(new Pose2d(-5,-11,Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-14.5,-17, Math.toRadians(169.2)))
                                .lineToLinearHeading(new Pose2d(-55,-20,Math.toRadians(270)))

                                .build()
                );
        RoadRunnerBotEntity Autonomia5 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 50, 5.69, 5.69, 7)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(StartPositionRight)
                                .lineToLinearHeading(new Pose2d(35,-20,Math.toRadians(10)))
                                .lineTo(new  Vector2d(33, -18))
                                .splineToSplineHeading(new Pose2d(-15,-17,Math.toRadians(180)),Math.toRadians(180))
                                .lineTo(new Vector2d(-30,-17))
                                .lineToLinearHeading(new Pose2d(-35,-20,Math.toRadians(167)))
                                .build()
                );
        RoadRunnerBotEntity Autonomia7 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 50, 5.69, 5.69, 7)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(StartPositionLeft)
                                // .setReversed(true)
                                .back(43)
                                .splineToSplineHeading(new Pose2d(-27.5,-5.5,Math.toRadians(225)),Math.toRadians(35))
                                .waitSeconds(1)
                                .lineTo(new Vector2d(-31,-7.5))
                                .splineToSplineHeading(new Pose2d(-55,-13,Math.toRadians(180)),Math.toRadians(180))
                                // .waitSeconds(3)
                                .lineTo(new Vector2d(-65,-13))
                                .waitSeconds(1)
                                .lineTo(new Vector2d(-40,-13))
                                .splineToSplineHeading(new Pose2d(-29,-20,Math.toRadians(125)),Math.toRadians(315))
                                .waitSeconds(1)
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
                                /*.lineTo(new Vector2d(33,-10))
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
                                .lineTo(new Vector2d(33,-7.5))*/
                                .build()
                );
        RoadRunnerBotEntity Carte = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 50, 5.69, 5.69, 7)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(StartPositionLeft)
                                .lineToLinearHeading(new Pose2d(-34.5,-14,Math.toRadians(217)))
                                .build()
                );
        RoadRunnerBotEntity Traiectorii = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 50, 5.69, 5.69, 7)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(StartPositionRight)
                                //.lineTo(new Vector2d(-33,-60))
                                .lineTo(new Vector2d(25,-59))
                                .splineToConstantHeading(new Vector2d(13,-50),Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(13,-35,Math.toRadians(13)),Math.toRadians(90))
                                .lineTo(new Vector2d(13,-20))
                                //.splineToConstantHeading(new Vector2d(17,-17),Math.toRadians(315))
                                //.splineToConstantHeading(new Vector2d(17,-17),Math.toRadians(0))
                                .build()
                );
        RoadRunnerBotEntity South_1_8 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                 .setConstraints(60, 50, 5.69, 5.69, 7)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(StartPositionRight)
                                .strafeRight(1)
                                .splineToSplineHeading(new Pose2d(10,-40,Math.toRadians(180)),Math.toRadians(180))
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(South_1_8)
                .start();
    }
}