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
    public double Kp = 0.02;
    public double Ki = 0.1;
    public double Kd = 0;
    public double Kg = 0;
    public double maxSpeed = 1;
    LiftStatus CurrentStatus = START, PreviousStatus = START;
    SimplePIDController LiftColectarePID = null;
    /// pe DreaptaLift am encoder
    int basePosition = 0 , highPosition = 2425;
    public LiftController()
    {
        LiftColectarePID = new SimplePIDController(Kp,Ki,Kd);
        LiftColectarePID.targetValue=basePosition;
        LiftColectarePID.maxOutput = maxSpeed;
    }
    void update (RobotMap Robotel, int LiftPosition)
    {
        double powerLift = LiftColectarePID.update(LiftPosition) + Kg;
        powerLift = Math.max(-1,Math.min(powerLift,1));
        Robotel.stangaLift.setPower(powerLift);
        Robotel.dreaptaLift.setPower(powerLift);
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
