package org.firstinspires.ftc.teamcode.drive.OGCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.OGCode.SimplePIDController;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "Testers")
public class DLSColectareEncoderTest extends LinearOpMode {
    public static double DISTANCE = 60; // in
    public static double Kp = 0.0055;
    public static double Ki = 0.05;
    public static double Kd = 0.0002;
    public static double Kg = 0;
    public static double maxSpeed = 0.9;
    public static double RetractedPosition = 0 , ExtendedPosition = 250;
    int TargetLift = 0;
    ElapsedTime timerPID = new ElapsedTime();

    @Override

    public void runOpMode() throws InterruptedException {
        List <LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        ElapsedTime changePositions = new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        RobotMap robot = new RobotMap(hardwareMap);
        SimplePIDController hello = new SimplePIDController(Kp,Ki,Kd);
        robot.left4Bar.setPosition(0.4);
        robot.right4Bar.setPosition(0.4);
        waitForStart();

        if (isStopRequested()) return;


        telemetry.update();
        hello.targetValue = RetractedPosition;
        while (!isStopRequested() && opModeIsActive())
        {
            int ColectarePosition = robot.motorColectare.getCurrentPosition();
            double stangaLiftPower = robot.motorColectare.getPower();
            double powerColectare = hello.update(ColectarePosition);
            powerColectare = Math.max(-1,Math.min(powerColectare,1));
            robot.motorColectare.setPower(powerColectare);

            if (changePositions.seconds()>4)
            {
                if (hello.targetValue == RetractedPosition )
                {
                    hello.targetValue = ExtendedPosition;
                }
                else
                {
                    hello.targetValue = RetractedPosition;
                }
                changePositions.reset();
            }

            telemetry.addData("ColectareEncoder", ColectarePosition);
            telemetry.addData("powerColectare", powerColectare);
            telemetry.addData("TargetLift",hello.targetValue);
            telemetry.addData("Error", hello.measuredError(ColectarePosition));
            if (Kp!=hello.p || Kd!=hello.d || Ki!=hello.i || maxSpeed !=hello.maxOutput )
            {
                hello.p = Kp;
                hello.d = Kd;
                hello.i = Ki;
                hello.maxOutput = maxSpeed;
            }

            telemetry.update();
        }
    }
}