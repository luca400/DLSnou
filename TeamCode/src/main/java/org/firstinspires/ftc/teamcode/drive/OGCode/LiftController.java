package org.firstinspires.ftc.teamcode.drive.OGCode;

import static org.firstinspires.ftc.teamcode.drive.OGCode.LiftController.LiftStatus.START;

public class LiftController {
    public enum LiftStatus
    {
        START,
        HIGH,
        LOW,
        MID,
        BASE,
    }
    public double CurrentSpeed=0;
    public double Kp = 0.009;
    public double Ki = 0.02;
    public double Kd = 0;
    public double Kg = 0;
    public double maxSpeed = 1;
    public static LiftStatus CurrentStatus = START, PreviousStatus = START;
    SimplePIDController LiftColectarePID = null;
    /// pe DreaptaLift am encoder
    int basePosition = 0;
    int lowPosition = 255;
    int midPosition = 590;
    int highPosition = 975;
    public int CurrentPosition = 0;
    public LiftController()
    {
        LiftColectarePID = new SimplePIDController(Kp,Ki,Kd);
        LiftColectarePID.targetValue=basePosition;
        LiftColectarePID.maxOutput = maxSpeed;
    }
    public void update(RobotMap Robotel, int LiftPosition, SigurantaLiftController sigurantaLiftController, double CurrentVoltage)
    {
        CurrentPosition = LiftPosition;
        double powerLift = LiftColectarePID.update(LiftPosition) + Kg;
        powerLift = Math.max(-1,Math.min(powerLift* 14 / CurrentVoltage,1));
        CurrentSpeed=powerLift;
        Robotel.stangaLift.setPower(powerLift);
        Robotel.dreaptaLift.setPower(powerLift);
        if (CurrentStatus != PreviousStatus)
        {
            switch (CurrentStatus)
            {
                case BASE:
                {
                    LiftColectarePID.targetValue = basePosition;
                    sigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.TRANSFER;
                    break;
                }
                case HIGH:
                {
                    sigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.JUNCTION;
                    LiftColectarePID.targetValue = highPosition;
                    break;
                }
                case MID:
                {
                    sigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.JUNCTION;
                    LiftColectarePID.targetValue = midPosition;
                    break;
                }
                case LOW:
                {
                    sigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.JUNCTION;
                    LiftColectarePID.targetValue = lowPosition;
                    break;
                }
            }
        }
        PreviousStatus = CurrentStatus;
    }
}
