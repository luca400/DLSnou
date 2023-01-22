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
    double transfer_Position = 1 , junction_Position = 0.45;
    public void update(RobotMap Robotel)
    {
        if (CurrentStatus!=PreviousStatus)
        {
            switch (CurrentStatus)
            {
                case TRANSFER:
                {
                    Robotel.servoLift.setPosition(transfer_Position);
                    break;
                }
                case JUNCTION:
                {
                    Robotel.servoLift.setPosition(junction_Position);
                    break;
                }
            }
        }
        PreviousStatus = CurrentStatus;
    }
}
