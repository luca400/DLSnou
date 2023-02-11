package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class TrajectoryTester extends LinearOpMode {
    public static double DISTANCE = 60; // in
    public static double x_CYCLING_POSITION = 33, y_CYCLING_POSITION = -3, Angle_CYCLING_POSITION = 345;
    public static double x_SSH = 10 ,y_SSH = -10 , Angle_SSH = 180, Tanget_Angle_SSH = 180;
    public static double x_LINETO = -29 , y_LINETO = -10;
    public static double x_LLH = -33 , y_LLH = -3, Angle_LLH = 192;

    public static double x_CYCLING_POSITION_LEFT = -33, y_CYCLING_POSITION_LEFT = -3, Angle_CYCLING_POSITION_LEFT = 192;
    public static double x_SSH_LEFT = -10 ,y_SSH_LEFT = -10 , Angle_SSH_LEFT = 180, Tanget_Angle_SSH_LEFT = 180;
    public static double x_LINETO_LEFT = 29 , y_LINETO_LEFT = -10;
    public static double x_LLH_LEFT = 33 , y_LLH_LEFT = -3, Angle_LLH_LEFT = 345;
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d StartPositionRight = new Pose2d(35,-60,Math.toRadians(270));
        Pose2d StartPositionLeft = new Pose2d(-35,-60,Math.toRadians(270));
        drive.setPoseEstimate(StartPositionRight);

        TrajectorySequence AutonomiaDreapta = drive.trajectorySequenceBuilder(StartPositionRight)
                .lineToLinearHeading(new Pose2d(x_CYCLING_POSITION,y_CYCLING_POSITION,Math.toRadians(Angle_CYCLING_POSITION)))
                .splineToSplineHeading(new Pose2d(x_SSH,y_SSH,Math.toRadians(Angle_SSH)),Math.toRadians(Tanget_Angle_SSH))
                .lineTo(new Vector2d(x_LINETO,y_LINETO))
                .lineToLinearHeading(new Pose2d(x_LLH,y_LLH,Math.toRadians(Angle_LLH)))
                .build();
        TrajectorySequence AutonomiaStanga = drive.trajectorySequenceBuilder(StartPositionLeft)
                .lineToLinearHeading(new Pose2d(x_CYCLING_POSITION_LEFT,y_CYCLING_POSITION_LEFT,Math.toRadians(Angle_CYCLING_POSITION_LEFT)))
                .splineToSplineHeading(new Pose2d(x_SSH_LEFT,y_SSH_LEFT,Math.toRadians(Angle_SSH_LEFT)),Math.toRadians(Tanget_Angle_SSH_LEFT))
                .lineTo(new Vector2d(x_LINETO_LEFT,y_LINETO_LEFT))
                .lineToLinearHeading(new Pose2d(x_LLH_LEFT,y_LLH_LEFT,Math.toRadians(Angle_LLH_LEFT)))
                .build();
        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(AutonomiaDreapta);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}
