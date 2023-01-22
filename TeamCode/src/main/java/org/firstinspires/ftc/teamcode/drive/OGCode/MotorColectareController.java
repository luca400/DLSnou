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
    }
    public static double Kp = 0.0055;
    public static double Ki = 0.05;
    public static double Kd = 0.0002;
    public static double maxSpeed = 0.9;
    public static MotorColectare CurrentStatus = INITIALIZED,  PreviousStatus = INITIALIZED;
    SimplePIDController MotorColectarePID = null;
    int extendedPosition = 300 , retractedPosition = -65;
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
