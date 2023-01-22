package org.firstinspires.ftc.teamcode.drive.OGCode;

import static org.firstinspires.ftc.teamcode.drive.OGCode.RobotController.RobotControllerStatus.INTER_GO_COLLECT;
import static org.firstinspires.ftc.teamcode.drive.OGCode.RobotController.RobotControllerStatus.INTER_GO_PLACE;
import static org.firstinspires.ftc.teamcode.drive.OGCode.RobotController.RobotControllerStatus.START;

import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotController {

    public enum RobotControllerStatus
    {
        START,
        GO_COLLECT,
        INTER_GO_COLLECT,
        GO_PLACE,
        INTER_GO_PLACE,
    }
    public static RobotControllerStatus CurrentStatus = START, PreviousStatus = START;
    ElapsedTime timerGO_COLLECT = new ElapsedTime() , timerGO_PLACE = new ElapsedTime() ,timeCOLLECT_RAPID_FIRE = new ElapsedTime();
    public void update(ServoLiftController servoLiftController, Servo4BarController servo4BarController, MotorColectareController motorColectareController, CloseClawController closeClawController, TurnClawController turnClawController)
    {
        if (PreviousStatus != CurrentStatus || CurrentStatus == INTER_GO_COLLECT || CurrentStatus == INTER_GO_PLACE)
        {
            switch (CurrentStatus)
            {

                case GO_COLLECT:
                {
                    timerGO_COLLECT.reset();
                    servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.COLLECT_DRIVE;
                    CurrentStatus = INTER_GO_COLLECT;
                    break;
                }
                case INTER_GO_COLLECT:
                {
                    turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.COLLECT;
                    if (timerGO_COLLECT.seconds()>0.2)
                    {
                        turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.COLLECT;
                        closeClawController.CurrentStatus = CloseClawController.closeClawStatus.OPEN;
                    }
                    if (timerGO_COLLECT.seconds()>0.5)
                    {
                        closeClawController.CurrentStatus = CloseClawController.closeClawStatus.OPEN;
                        servoLiftController.CurrentStatus = ServoLiftController.ServoLiftStatus.TRANSFER;
                        CurrentStatus = START;
                    }
                    break;
                }
                case GO_PLACE:
                {
                    timerGO_PLACE.reset();
                    servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.PLACE_CONE;
                    CurrentStatus = INTER_GO_PLACE;
                    break;
                }
                case INTER_GO_PLACE:
                {
                    if (timerGO_PLACE.seconds()>0.3)
                    {
                        turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.PLACE;
                    }
                    if (timerGO_PLACE.seconds()>1.5)
                    {
                        closeClawController.CurrentStatus = CloseClawController.closeClawStatus.OPEN;
                        servoLiftController.CurrentStatus = ServoLiftController.ServoLiftStatus.JUNCTION;
                        servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.DRIVE_POSITION;
                        CurrentStatus = START;
                    }
                    break;
                }
            }
        }
        PreviousStatus = CurrentStatus;
    }
}
