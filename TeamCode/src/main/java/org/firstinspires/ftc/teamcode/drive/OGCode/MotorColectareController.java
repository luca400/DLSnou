package org.firstinspires.ftc.teamcode.drive.OGCode;

import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.EXTENDED;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.EXTENDED_1050;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.EXTENDED_2050;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.INITIALIZED;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.RETRACTED;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
public class MotorColectareController {
    public enum MotorColectare
    {
        INITIALIZED,
        EXTENDED,
        RETRACTED,
        RETRACTED_0,
        EXTENDED_FAST,
        EXTENDED_5_1,
        EXTENDED_10_1,
        EXTENDED_10_1_SOUTH,
        EXTENDED_COMMONHIGHINTERFERENCE,
        EXTENDED_SOUTH_LEFT,
        CLOSE_TO_EXTENDED_FIRST_CONE,
        EXTENDED_FIRST_CONE,
        EXTENDED_1050,
        HALF_WAY,
        THREE_WAY,
        EXTENDED_SOUTH,
        EXTENDED_2050,
    }
    public static double Kp = 0.003;
    public static double Ki = 0.002;
    public static double Kd = 0.00011;
    public static double maxSpeed = 1;
    public static MotorColectare CurrentStatus = INITIALIZED,  PreviousStatus = INITIALIZED;
    SimplePIDController MotorColectarePID = null;
    public static double vMax = 0, AccMax = 0, JerkMax =0 , EndPos = 2020 , CurrentPosition = 0;
    public static int extendedPosition = 700 , retractedPosition = -30, extendedDrive = 875;
    public static int extended5_1Autonomy_5= 465, extended5_1Autonomy_4 = 450   ,extended5_1Autonomy_3 = 440,extended5_1Autonomy_2 = 430,extended5_1Autonomy_1 = 430;
    public static int extended10_1Autonomy_5= 465, extended10_1Autonomy_4 = 450,extended10_1Autonomy_3 = 440,extended10_1Autonomy_2 = 430,extended10_1Autonomy_1 = 430;
    public static int NrConAuto = 5;
    public MotorColectareController()
    {
        MotorColectarePID = new SimplePIDController(Kp,Ki,Kd);
        MotorColectarePID.targetValue=retractedPosition;
        MotorColectarePID.maxOutput = maxSpeed;
    }
    public void update(RobotMap Robotel, int ColectarePosition, double PowerCap, double CurrentVoltage)
    {
        CurrentPosition = ColectarePosition;
        double powerColectare = MotorColectarePID.update(ColectarePosition);
        powerColectare = Math.max(-PowerCap,Math.min(powerColectare,PowerCap));
        Robotel.motorColectareStanga.setPower(powerColectare);
        Robotel.motorColectareDreapta.setPower(powerColectare);
        if (PreviousStatus != CurrentStatus || CurrentStatus == EXTENDED_1050 || CurrentStatus == EXTENDED || CurrentStatus == EXTENDED_2050)
        {
            switch (CurrentStatus)
            {
                case RETRACTED:
                {
                    MotorColectarePID.targetValue = retractedPosition;
                    MotorColectarePID.maxOutput = 1;
                    break;
                }
                case RETRACTED_0:
                {
                    MotorColectarePID.targetValue = 0;
                    MotorColectarePID.maxOutput = 1;
                    break;
                }
                case EXTENDED_1050:
                {
                    MotorColectarePID.maxOutput = 1;
                    MotorColectarePID.targetValue = extendedDrive;
                    break;
                }
                case EXTENDED:
                {
                    MotorColectarePID.maxOutput = 1;
                    MotorColectarePID.targetValue = 700;
                    break;
                }
                case EXTENDED_SOUTH:
                {
                    MotorColectarePID.maxOutput = 1;
                    switch (NrConAuto) {
                        case 0:
                        {
                            MotorColectarePID.targetValue = 790;
                            break;
                        }
                        case 1: {
                            MotorColectarePID.targetValue = 710;
                            break;
                        }
                        case 2: {
                            MotorColectarePID.targetValue = 730;
                            break;
                        }
                        case 3: {
                            MotorColectarePID.targetValue = 760;
                            break;
                        }
                        case 4: {
                            MotorColectarePID.targetValue = 770;
                            break;
                        }
                        case 5:
                        {
                            MotorColectarePID.targetValue = 800;
                            break;
                        }
                    }
                    break;
                }
                case HALF_WAY:
                {
                    MotorColectarePID.maxOutput = 0.4;
                    MotorColectarePID.targetValue = 420;
                    break;
                }
                case THREE_WAY:
                {
                    MotorColectarePID.maxOutput = 0.4;
                    MotorColectarePID.targetValue = 550;
                    break;
                }
                case EXTENDED_SOUTH_LEFT:
                {
                    MotorColectarePID.maxOutput = 0.7;
                    switch (NrConAuto) {
                        case 0:
                        {
                            MotorColectarePID.targetValue = 555;
                            break;
                        }
                        case 1: {
                            MotorColectarePID.targetValue = 555;
                            break;
                        }
                        case 2: {
                            MotorColectarePID.targetValue = 575;
                            break;
                        }
                        case 3: {
                            MotorColectarePID.targetValue = 595;
                            break;
                        }
                        case 4: {
                            MotorColectarePID.targetValue = 595;
                            break;
                        }
                    }
                    break;
                }
                case EXTENDED_COMMONHIGHINTERFERENCE:
                {
                    MotorColectarePID.maxOutput = 0.5;
                    switch (NrConAuto) {
                        case 1: {
                            MotorColectarePID.targetValue = 240;
                            break;
                        }
                        case 2: {
                            MotorColectarePID.targetValue = 260;
                            break;
                        }
                        case 3: {
                            MotorColectarePID.targetValue = 260;
                            break;
                        }
                        case 4: {
                            MotorColectarePID.targetValue = 260;
                            break;
                        }
                        case 5: {
                            MotorColectarePID.targetValue = 475;
                            break;
                        }
                    }
                    break;
                }
                case EXTENDED_5_1:
                {
                    MotorColectarePID.maxOutput = 1;
                    switch (NrConAuto)
                    {
                        case 1:
                        {
                            MotorColectarePID.targetValue = extended5_1Autonomy_1;
                            break;
                        }
                        case 2:
                        {
                            MotorColectarePID.targetValue = extended5_1Autonomy_2;
                            break;
                        }
                        case 3:
                        {
                            MotorColectarePID.targetValue = extended5_1Autonomy_3;
                            break;
                        }
                        case 4:
                        {
                            MotorColectarePID.targetValue = extended5_1Autonomy_4;
                            break;
                        }
                        case 5:
                        {
                            MotorColectarePID.targetValue = extended5_1Autonomy_5;
                            break;
                        }
                    }
                    break;
                }
                case EXTENDED_10_1:
                {
                    MotorColectarePID.maxOutput = 1;
                    switch (NrConAuto)
                    {
                        case 1:
                        {
                            MotorColectarePID.targetValue = extended10_1Autonomy_1;
                            break;
                        }
                        case 2:
                        {
                            MotorColectarePID.targetValue = extended10_1Autonomy_2;
                            break;
                        }
                        case 3:
                        {
                            MotorColectarePID.targetValue = extended10_1Autonomy_3;
                            break;
                        }
                        case 4:
                        {
                            MotorColectarePID.targetValue = extended10_1Autonomy_4;
                            break;
                        }
                        case 5:
                        {
                            MotorColectarePID.targetValue = extended10_1Autonomy_5;
                            break;
                        }
                    }
                    break;
                }
                case EXTENDED_10_1_SOUTH:
                {
                    MotorColectarePID.maxOutput = 1;
                    switch (NrConAuto)
                    {
                        case 1:
                        {
                            MotorColectarePID.targetValue = 925;
                            break;
                        }
                        case 2:
                        {
                            MotorColectarePID.targetValue = 925;
                            break;
                        }
                        case 3:
                        {
                            MotorColectarePID.targetValue = 925;
                            break;
                        }
                        case 4:
                        {
                            MotorColectarePID.targetValue = 925;
                            break;
                        }
                        case 5:
                        {
                            MotorColectarePID.targetValue = 925;
                            break;
                        }
                    }
                    break;
                }
                case EXTENDED_FAST:
                {
                    MotorColectarePID.maxOutput = 1;
                    MotorColectarePID.targetValue = extendedPosition;
                    break;
                }
                case EXTENDED_2050:
                {
                    MotorColectarePID.maxOutput = 1;
                    MotorColectarePID.targetValue = 700;
                    break;
                }
            }
        }
        PreviousStatus = CurrentStatus;
    }
}
