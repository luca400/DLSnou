package org.firstinspires.ftc.teamcode.drive.OGCode;

import static org.firstinspires.ftc.teamcode.drive.OGCode.TurnClawController.TurnClawStatus.INIT;
import static org.firstinspires.ftc.teamcode.drive.OGCode.TurnClawController.TurnClawStatus.PLACE;

public class TurnClawController {
    public enum TurnClawStatus
    {
        INIT,
        COLLECT,
        PLACE,
    }
    public static TurnClawStatus CurrentStatus = INIT , PreviousStatus = INIT;
    double pozTurnClaw_COLLECT=0.70, pozTurnClaw_PLACE = 0.05;
    public void update(RobotMap Robotel)
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
