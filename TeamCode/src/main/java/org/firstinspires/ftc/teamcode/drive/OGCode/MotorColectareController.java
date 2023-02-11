package org.firstinspires.ftc.teamcode.drive.OGCode;

import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.EXTENDED;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.EXTENDED_600;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.INITIALIZED;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.RETRACTED;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MotorColectareController {
    public enum MotorColectare
    {
        INITIALIZED,
        EXTENDED,
        RETRACTED,
        CLOSE_TO_EXTENDED_FIRST_CONE,
        EXTENDED_FIRST_CONE,
        EXTENDED_600
    }
    public static double Kp = 0.0012;
    public static double Ki = 0.0015;
    public static double Kd = 0;
    public static double maxSpeed = 0.75;
    public static MotorColectare CurrentStatus = INITIALIZED,  PreviousStatus = INITIALIZED;
    SimplePIDController MotorColectarePID = null;
    public static double vMax = 50000, AccMax = 50000, JerkMax =50000 , EndPos = 2020 , CurrentPosition = 0;
    public static int extendedPosition = 2020 , retractedPosition = -75;
    ElapsedTime timer600 = new ElapsedTime();
    ElapsedTime timer2020 = new ElapsedTime();
    MotionProfile profile2020 = MotionProfileGenerator.generateSimpleMotionProfile(
            new MotionState(0, 0, 0),
            new MotionState(2220, 0, 0),
            vMax,
            AccMax,
            JerkMax
    );
    MotionProfile profile600 = MotionProfileGenerator.generateSimpleMotionProfile(
            new MotionState(0, 0, 0),
            new MotionState(600, 0, 0),
            vMax,
            AccMax,
            JerkMax
    );
    public MotorColectareController()
    {
        MotorColectarePID = new SimplePIDController(Kp,Ki,Kd);
        MotorColectarePID.targetValue=retractedPosition;
        MotorColectarePID.maxOutput = maxSpeed;
    }
    public void update(RobotMap Robotel, int ColectarePosition, double PowerCap)
    {
        CurrentPosition = ColectarePosition;
        double powerColectare = MotorColectarePID.update(ColectarePosition);
        powerColectare = Math.max(-PowerCap,Math.min(powerColectare,PowerCap));
        Robotel.motorColectare.setPower(powerColectare);
        if (PreviousStatus != CurrentStatus || CurrentStatus == EXTENDED_600 || CurrentStatus == EXTENDED)
        {
            switch (CurrentStatus)
            {
                case RETRACTED:
                {
                    MotorColectarePID.targetValue = retractedPosition;
                    MotorColectarePID.maxOutput = 1;
                    break;
                }
                case EXTENDED_600:
                {
                    if (PreviousStatus != CurrentStatus)
                    {
                        timer600.reset();
                    }
                    MotorColectarePID.maxOutput = 0.25;
                    MotorColectarePID.targetValue = profile600.get(timer600.seconds()).getX();
                    break;
                }
                case EXTENDED:
                {
                    if (PreviousStatus != CurrentStatus)
                    {
                        timer2020.reset();
                    }
                    MotorColectarePID.maxOutput = 1;
                    MotorColectarePID.targetValue = profile2020.get(timer2020.seconds()).getX();
                    break;
                }
            }
        }
        PreviousStatus = CurrentStatus;
    }
}
