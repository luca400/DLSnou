package org.firstinspires.ftc.teamcode.drive.OGCode;

import org.checkerframework.checker.units.qual.Current;

public class CloseClawController {

    enum closeClawStatus
    {
        INIT,
        CLOSED,
        OPEN,
    }
    closeClawStatus CurrentStatus = closeClawStatus.INIT,  PreviousStatus = closeClawStatus.INIT;
    double pozOpenClaw = 0.6, pozCloseClaw = 0.8;

    void update(RobotMap Robotel)
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
            }
        }
        PreviousStatus = CurrentStatus;
    }
}
