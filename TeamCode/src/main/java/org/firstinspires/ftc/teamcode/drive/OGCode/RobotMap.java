package org.firstinspires.ftc.teamcode.drive.OGCode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

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
import org.firstinspires.ftc.teamcode.util.Encoder;

public class RobotMap {

    public Servo left4Bar = null;
    public Servo right4Bar = null;
    Servo turnClaw = null;
    Servo closeClaw = null;
    Servo servoLift = null;
    Servo sigurantaLift = null;
    Servo angle4Bar = null;
    public DcMotorEx motorColectare = null;
    public DcMotorEx dreaptaLift = null;
    DcMotorEx stangaLift = null;
    public Encoder encoderMotorColectare = null;

    public RobotMap(HardwareMap Init)
    {
        left4Bar = Init.get(Servo.class, "left4Bar");
        right4Bar = Init.get(Servo.class,"right4Bar");
        turnClaw = Init.get(Servo.class, "turnClaw");
        closeClaw = Init.get(Servo.class, "closeClaw");
        servoLift = Init.get(Servo.class,"servoLift");
        sigurantaLift = Init.get(Servo.class,"sigurantaLift");
        angle4Bar = Init.get(Servo.class,"angle4Bar");

        motorColectare = Init.get(DcMotorEx.class, "motorColectare");
        stangaLift = Init.get(DcMotorEx.class, "stangaLift");
        dreaptaLift = Init.get(DcMotorEx.class, "dreaptaLift");


        right4Bar.setDirection(Servo.Direction.REVERSE);
        stangaLift.setDirection(DcMotorSimple.Direction.REVERSE);

        dreaptaLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stangaLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stangaLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dreaptaLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorColectare.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorColectare.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderMotorColectare = new Encoder(Init.get(DcMotorEx.class, "motorColectare"));
        //encoderMotorColectare.setDirection(Encoder.Direction.REVERSE);
        motorColectare.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorColectare.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}