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
    public double Kp = 0.01;
    public double Ki = 0.1;
    public double Kd = 0;
    public double Kg = 0;
    public double maxSpeed = 1;
    public static LiftStatus CurrentStatus = START, PreviousStatus = START;
    SimplePIDController LiftColectarePID = null;
    /// pe DreaptaLift am encoder
    int basePosition = 0 , lowPosition = 620, midPosition = 1500,  highPosition = 2425;
    public LiftController()
    {
        LiftColectarePID = new SimplePIDController(Kp,Ki,Kd);
        LiftColectarePID.targetValue=basePosition;
        LiftColectarePID.maxOutput = maxSpeed;
    }
    public void update(RobotMap Robotel, int LiftPosition, SigurantaLiftController sigurantaLiftController, ServoLiftController servoLiftController)
    {
        double powerLift = LiftColectarePID.update(LiftPosition) + Kg;
        powerLift = Math.max(-1,Math.min(powerLift,1));
        Robotel.stangaLift.setPower(powerLift);
        Robotel.dreaptaLift.setPower(powerLift);
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
                    if (LiftColectarePID.targetValue-LiftPosition < 1200)
                    {
                        servoLiftController.CurrentStatus = ServoLiftController.ServoLiftStatus.JUNCTION;
                    }
                    break;
                }
                case MID:
                {
                    sigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.JUNCTION;
                    LiftColectarePID.targetValue = midPosition;
                    if (LiftColectarePID.targetValue-LiftPosition < 750)
                    {
                        servoLiftController.CurrentStatus = ServoLiftController.ServoLiftStatus.JUNCTION;
                    }
                    break;
                }
                case LOW:
                {
                    sigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.JUNCTION;
                    LiftColectarePID.targetValue = lowPosition;
                    if (LiftColectarePID.targetValue-LiftPosition < 300)
                    {
                        servoLiftController.CurrentStatus = ServoLiftController.ServoLiftStatus.JUNCTION;
                    }
                    break;
                }
            }
    }
}
