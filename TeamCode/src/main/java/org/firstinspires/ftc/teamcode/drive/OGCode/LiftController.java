package org.firstinspires.ftc.teamcode.drive.OGCode;

import static org.firstinspires.ftc.teamcode.drive.OGCode.LiftController.LiftStatus.START;

public class LiftController {
    enum LiftStatus
    {
        START,
        HIGH,
        LOW,
        MEDIUM,
        BASE,
    }
    public double Kp = 0.004;
    public double Ki = 0;
    public double Kd = 0;
    public double maxSpeed = 0.5;
    LiftStatus CurrentStatus = START, PreviousStatus = START;
    SimplePIDController LiftColectarePID = null;
    /// pe DreaptaLift am encoder
    int basePosition = 0 , highPosition = 1500;
    public LiftController()
    {
        LiftColectarePID = new SimplePIDController(Kp,Ki,Kd);
        LiftColectarePID.targetValue=basePosition;
        LiftColectarePID.maxOutput = maxSpeed;
    }
    void update (RobotMap Robotel, int LiftPosition)
    {
        double powerLift = LiftColectarePID.update(LiftPosition);
        powerLift = Math.max(-1,Math.min(powerLift,1));
        Robotel.motorColectare.setPower(powerLift);
        if (PreviousStatus != CurrentStatus)
        {
            switch (CurrentStatus)
            {
                case BASE:
                {
                    LiftColectarePID.targetValue = basePosition;
                    break;
                }
                case HIGH:
                {
                    LiftColectarePID.targetValue = highPosition;
                    break;
                }
            }
        }
        PreviousStatus = CurrentStatus;
    }
}
