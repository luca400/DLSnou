package org.firstinspires.ftc.teamcode.drive.OGCode;

import android.hardware.Sensor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;


import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class RobotMap {

    Servo left4Bar = null;
    Servo right4Bar = null;
    Servo turnCLaw = null;
    Servo closeClaw = null;
    DcMotorEx motorColectare = null;
    DcMotorEx dreaptaLift = null;
    DcMotorEx stangaLift = null;

    public RobotMap(HardwareMap Init)
    {
        left4Bar = Init.get(Servo.class, "left4Bar");
        right4Bar = Init.get(Servo.class,"right4Bar");
        turnCLaw = Init.get(Servo.class, "turnClaw");
        closeClaw = Init.get(Servo.class, "closeClaw");

        motorColectare = Init.get(DcMotorEx.class, "motorColectare");
        stangaLift = Init.get(DcMotorEx.class, "stangaLift");
        dreaptaLift = Init.get(DcMotorEx.class, "dreaptaLift");



        right4Bar.setDirection(Servo.Direction.REVERSE);
        dreaptaLift.setDirection(DcMotorSimple.Direction.REVERSE);

        dreaptaLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stangaLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stangaLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dreaptaLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorColectare.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorColectare.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorColectare.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorColectare.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}