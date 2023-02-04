package org.firstinspires.ftc.teamcode.drive.OGCode;

import org.checkerframework.checker.units.qual.Current;

public class CloseClawController {

    public enum closeClawStatus
    {
        INIT,
        CLOSED,
        OPEN,
        OPEN_COLLECT,
    }
    public static closeClawStatus CurrentStatus = closeClawStatus.INIT,  PreviousStatus = closeClawStatus.INIT;
    double pozOpenClaw = 0.75, pozCloseClaw = 1 ,pozOpenClawCollect = 0.75;

    public void update(RobotMap Robotel)
    {
        if (PreviousStatus!= CurrentStatus)
        {
            switch(CurrentStatus)
            {
                case CLOSED:
                {
                    Robotel.closeClaw.setPosition(pozCloseClaw);
                    break;
                }
                case OPEN:
                {
                    Robotel.closeClaw.setPosition(pozOpenClaw);
                    break;
                }
                case OPEN_COLLECT:
                {
                    Robotel.closeClaw.setPosition(pozOpenClawCollect);
                    break;
                }
            }
        }
        PreviousStatus = CurrentStatus;
    }
}
