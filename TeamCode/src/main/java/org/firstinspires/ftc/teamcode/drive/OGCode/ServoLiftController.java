package org.firstinspires.ftc.teamcode.drive.OGCode;

import static org.firstinspires.ftc.teamcode.drive.OGCode.ServoLiftController.ServoLiftStatus.START;
import static org.firstinspires.ftc.teamcode.drive.OGCode.ServoLiftController.ServoLiftStatus.TRANSFER;

import com.qualcomm.robotcore.robot.Robot;

import org.checkerframework.checker.units.qual.Current;

public class ServoLiftController {
    public enum ServoLiftStatus
    {
        START,
        TRANSFER,
        JUNCTION,
    }
    public static ServoLiftStatus CurrentStatus = START, PreviousStatus = START;
    double transfer_Position = 0.985 , junction_Position = 0.45;
    public void update(RobotMap Robotel, SigurantaLiftController sigurantaLiftController)
    {
        if (CurrentStatus!=PreviousStatus)
        {
            switch (CurrentStatus)
            {
                case TRANSFER:
                {
                    sigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.TRANSFER;
                    Robotel.servoLift.setPosition(transfer_Position);
                    break;
                }
                case JUNCTION:
                {
                    sigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.JUNCTION;
                    Robotel.servoLift.setPosition(junction_Position);
                    break;
                }
            }
        }
        PreviousStatus = CurrentStatus;
    }
}
