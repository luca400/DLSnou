package org.firstinspires.ftc.teamcode.drive.OGCode;

import static org.firstinspires.ftc.teamcode.drive.OGCode.TurnClawController.TurnClawStatus.INIT;
import static org.firstinspires.ftc.teamcode.drive.OGCode.TurnClawController.TurnClawStatus.PLACE;

public class TurnClawController {
    enum TurnClawStatus
    {
        INIT,
        COLLECT,
        PLACE,
    }
    TurnClawStatus CurrentStatus = INIT , PreviousStatus = INIT;
    double pozTurnClaw_COLLECT=0.72, pozTurnClaw_PLACE = 0.05;
    void update(RobotMap Robotel)
    {
        if (PreviousStatus != CurrentStatus)
        {
            switch (CurrentStatus)
            {
                case PLACE:
                {
                    Robotel.turnClaw.setPosition(pozTurnClaw_PLACE);
                    break;
                }
                case COLLECT:
                {
                    Robotel.turnClaw.setPosition(pozTurnClaw_COLLECT);
                    break;
                }
            }
        }
        PreviousStatus = CurrentStatus;
    }
}
