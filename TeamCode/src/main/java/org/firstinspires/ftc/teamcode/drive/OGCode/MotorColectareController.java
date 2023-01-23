package org.firstinspires.ftc.teamcode.drive.OGCode;

import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.INITIALIZED;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.RETRACTED;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MotorColectareController {
    public enum MotorColectare
    {
        INITIALIZED,
        EXTENDED,
        RETRACTED,
        CLOSE_TO_EXTENDED_FIRST_CONE,
        EXTENDED_FIRST_CONE
    }
    public static double Kp = 0.0012;
    public static double Ki = 0.005;
    public static double Kd = 0;
    public static double maxSpeed = 0.75;
    public static MotorColectare CurrentStatus = INITIALIZED,  PreviousStatus = INITIALIZED;
    SimplePIDController MotorColectarePID = null;
    public static int extendedPosition = 2020 , retractedPosition = -75,closetoExtended = 1650,extendedFirstCone = 2000;
    public MotorColectareController()
    {
        MotorColectarePID = new SimplePIDController(Kp,Ki,Kd);
        MotorColectarePID.targetValue=retractedPosition;
        MotorColectarePID.maxOutput = maxSpeed;
    }
    public void update(RobotMap Robotel, int ColectarePosition, double PowerCap)
    {
        double powerColectare = MotorColectarePID.update(ColectarePosition);
        powerColectare = Math.max(-PowerCap,Math.min(powerColectare,PowerCap));
        Robotel.motorColectare.setPower(powerColectare);
        if (PreviousStatus != CurrentStatus)
        {
            switch (CurrentStatus)
            {
                case RETRACTED:
                {
                    MotorColectarePID.targetValue = retractedPosition;
                    break;
                }
                case CLOSE_TO_EXTENDED_FIRST_CONE:
                {
                    MotorColectarePID.targetValue = closetoExtended;
                    break;
                }
                case EXTENDED_FIRST_CONE:
                {
                    MotorColectarePID.targetValue = extendedFirstCone;
                    break;
                }
                case EXTENDED:
                {
                    MotorColectarePID.targetValue = extendedPosition;
                    break;
                }
            }
        }
        PreviousStatus = CurrentStatus;
    }
}
