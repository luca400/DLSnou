package org.firstinspires.ftc.teamcode.drive.OGCode;

import static org.firstinspires.ftc.teamcode.drive.OGCode.RobotController.RobotControllerStatus.INTER_GO_COLLECT;
import static org.firstinspires.ftc.teamcode.drive.OGCode.RobotController.RobotControllerStatus.INTER_GO_PLACE;
import static org.firstinspires.ftc.teamcode.drive.OGCode.RobotController.RobotControllerStatus.NOTHING;

import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotController {

    enum RobotControllerStatus
    {
        NOTHING,
        GO_COLLECT,
        INTER_GO_COLLECT,
        GO_PLACE,
        INTER_GO_PLACE,
    }
    RobotControllerStatus CurrentStatus = NOTHING, PreviousStatus = NOTHING;
    ElapsedTime timerGO_COLLECT = new ElapsedTime() , timerGO_PLACE = new ElapsedTime() ,timeCOLLECT_RAPID_FIRE = new ElapsedTime();
    void update(Servo4BarController servo4BarController, MotorColectareController motorColectareController, CloseClawController closeClawController, TurnClawController turnClawController)
    {
        if (PreviousStatus != CurrentStatus || CurrentStatus == INTER_GO_COLLECT || CurrentStatus == INTER_GO_PLACE)
        {
            switch (CurrentStatus)
            {
                case GO_COLLECT:
                {
                    timerGO_COLLECT.reset();
                    servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.COLLECT_DRIVE;
                    turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.COLLECT;
                    CurrentStatus = INTER_GO_COLLECT;
                    break;
                }
                case INTER_GO_COLLECT:
                {
                    if (closeClawController.CurrentStatus!= CloseClawController.closeClawStatus.CLOSED && timerGO_COLLECT.seconds()>0.2)
                    {
                        closeClawController.CurrentStatus = CloseClawController.closeClawStatus.CLOSED;
                    }
                    if (timerGO_COLLECT.seconds()>0.5)
                    {
                        closeClawController.CurrentStatus = CloseClawController.closeClawStatus.OPEN;
                        CurrentStatus = NOTHING;
                    }
                    break;
                }
                case GO_PLACE:
                {
                    timerGO_PLACE.reset();
                    servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.PLACE_CONE;
                    turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.PLACE;
                    CurrentStatus = INTER_GO_PLACE;
                    break;
                }
                case INTER_GO_PLACE:
                {
                    if (timerGO_PLACE.seconds()>1.5)
                    {
                        closeClawController.CurrentStatus = CloseClawController.closeClawStatus.OPEN;
                        CurrentStatus = NOTHING;
                    }
                    break;
                }
            }
        }
        PreviousStatus = CurrentStatus;
    }
}
