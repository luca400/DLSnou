package org.firstinspires.ftc.teamcode.drive.OGCode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.hardware.Sensor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
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
    public Servo turnClaw = null;
    Servo closeClaw = null;
    Servo servoLift = null;
    Servo sigurantaLift = null;
    Servo angle4Bar = null;
    public DcMotorEx motorColectareStanga = null;
    public DcMotorEx motorColectareDreapta = null;
    public DcMotorEx dreaptaLift = null;
    DcMotorEx stangaLift = null;
    public static int xAI = 300,yAI = 150,xBI = 320,yBI = 180;
    public RobotMap(HardwareMap Init)
    {
        left4Bar = Init.get(Servo.class, "left4Bar");
        right4Bar = Init.get(Servo.class,"right4Bar");

        turnClaw = Init.get(Servo.class, "turnClaw");
        closeClaw = Init.get(Servo.class, "closeClaw");
        servoLift = Init.get(Servo.class,"servoLift");
        sigurantaLift = Init.get(Servo.class,"sigurantaLift");
        angle4Bar = Init.get(Servo.class,"angle4Bar");

        motorColectareStanga = Init.get(DcMotorEx.class, "motorColectareStanga");
        motorColectareDreapta = Init.get(DcMotorEx.class, "motorColectareDreapta");
        stangaLift = Init.get(DcMotorEx.class, "stangaLift");
        dreaptaLift = Init.get(DcMotorEx.class, "dreaptaLift");


        right4Bar.setDirection(Servo.Direction.REVERSE);
        left4Bar.setDirection(Servo.Direction.REVERSE);

        stangaLift.setDirection(DcMotorSimple.Direction.REVERSE);

        dreaptaLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stangaLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stangaLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dreaptaLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorColectareStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorColectareStanga.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorColectareStanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        motorColectareDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorColectareDreapta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorColectareDreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorColectareDreapta.setDirection(DcMotorSimple.Direction.REVERSE);

    }
}