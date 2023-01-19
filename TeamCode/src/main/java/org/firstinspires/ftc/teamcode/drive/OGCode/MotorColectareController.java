package org.firstinspires.ftc.teamcode.drive.OGCode;

import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.INITIALIZED;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.RETRACTED;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MotorColectareController {
    enum MotorColectare
    {
        INITIALIZED,
        EXTENDED,
        RETRACTED,
    }
    public static double Kp = 0.0055;
    public static double Ki = 0.05;
    public static double Kd = 0.0002;
    public static double maxSpeed = 0.9;
    MotorColectare CurrentStatus = INITIALIZED,  PreviousStatus = INITIALIZED;
    SimplePIDController MotorColectarePID = null;
    int extendedPosition = 250 , retractedPosition = -65;
    public MotorColectareController()
    {
        MotorColectarePID = new SimplePIDController(Kp,Ki,Kd);
        MotorColectarePID.targetValue=retractedPosition;
        MotorColectarePID.maxOutput = maxSpeed;
    }
    void update(RobotMap Robotel ,int ColectarePosition)
    {
        double powerColectare = MotorColectarePID.update(ColectarePosition);
        powerColectare = Math.max(-1,Math.min(powerColectare,1));
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
