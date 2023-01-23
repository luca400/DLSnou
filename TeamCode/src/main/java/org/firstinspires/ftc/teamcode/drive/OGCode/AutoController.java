package org.firstinspires.ftc.teamcode.drive.OGCode;

import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoController.autoControllerStatus.COLLECT_RAPID_FIRE_AUTO;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoController.autoControllerStatus.COLLECT_RAPID_FIRE_INTER_AUTO;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoController.autoControllerStatus.COLLECT_RAPID_FIRE_INTER2_AUTO;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoController.autoControllerStatus.LIFT_CONE;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoController.autoControllerStatus.NOTHING;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.CLOSE_TO_EXTENDED_FIRST_CONE;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.EXTENDED;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.EXTENDED_FIRST_CONE;

import com.qualcomm.robotcore.util.ElapsedTime;

public class AutoController {
    public enum autoControllerStatus
    {
        NOTHING,
        COLLECT_RAPID_FIRE_AUTO,
        COLLECT_RAPID_FIRE_INTER_AUTO,
        COLLECT_RAPID_FIRE_INTER2_AUTO,
        STACK_LEVEL,
        LIFT_CONE,
        INITIALIZE
    }
    public static autoControllerStatus CurrentStatus = NOTHING, PreviousStatus = NOTHING;
    ElapsedTime timerCOLLECT_RAPID_FIRE2 = new ElapsedTime() ,timerCOLLECT_RAPID_FIRE1 = new ElapsedTime(),
            timerLIFT = new ElapsedTime(), timerStart = new ElapsedTime();
    int Cone_Stack_Level=5;
    double timerInter = 2,timeStart=0;
    public void update(TurnClawController turnClawController, ServoLiftController servoLiftController, LiftController liftController, Servo4BarController servo4BarController, RobotController robotController, CloseClawController closeClawController, MotorColectareController motorColectareController)
    {
        if (CurrentStatus!=PreviousStatus || CurrentStatus == LIFT_CONE || CurrentStatus == COLLECT_RAPID_FIRE_AUTO ||CurrentStatus ==  COLLECT_RAPID_FIRE_INTER_AUTO|| CurrentStatus == COLLECT_RAPID_FIRE_INTER2_AUTO)
        {
            switch (CurrentStatus)
            {
                case INITIALIZE:
                {
                    servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.INITIALIZE;
                    motorColectareController.CurrentStatus = MotorColectareController.MotorColectare.RETRACTED;
                    closeClawController.CurrentStatus = CloseClawController.closeClawStatus.CLOSED;
                    turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.COLLECT;
                    robotController.CurrentStatus = RobotController.RobotControllerStatus.START;
                    liftController.CurrentStatus = LiftController.LiftStatus.BASE;
                    servoLiftController.CurrentStatus = ServoLiftController.ServoLiftStatus.TRANSFER;
                    CurrentStatus = NOTHING;
                    break;
                }

                case COLLECT_RAPID_FIRE_AUTO:
                {
                    if (Cone_Stack_Level+1==5)
                    {
                        timerCOLLECT_RAPID_FIRE1.reset();
                        if (motorColectareController.CurrentStatus == MotorColectareController.MotorColectare.RETRACTED)
                        {
                            motorColectareController.CurrentStatus  = CLOSE_TO_EXTENDED_FIRST_CONE;
                        }
                        if (timerStart.seconds()>2)
                        {
                            robotController.CurrentStatus = RobotController.RobotControllerStatus.GO_COLLECT;
                            CurrentStatus = COLLECT_RAPID_FIRE_INTER_AUTO;
                        }
                    }
                    else
                    {
                        if (robotController.CurrentStatus == RobotController.RobotControllerStatus.START&&timerStart.seconds()>timeStart)
                        {
                            timerCOLLECT_RAPID_FIRE1.reset();
                            robotController.CurrentStatus = RobotController.RobotControllerStatus.GO_COLLECT;
                        }
                        if (timerStart.seconds()>timeStart+0.5)
                        {
                            motorColectareController.CurrentStatus = EXTENDED;
                            CurrentStatus = COLLECT_RAPID_FIRE_INTER_AUTO;
                        }
                    }
                    break;
                }
                case COLLECT_RAPID_FIRE_INTER_AUTO:
                {

                    if (servo4BarController.CurrentStatus == Servo4BarController.ServoStatus.COLLECT_DRIVE) timerInter = 1;
                    else if (servo4BarController.CurrentStatus == Servo4BarController.ServoStatus.INTERMEDIARY) timerInter = 1.5;


                    if (timerCOLLECT_RAPID_FIRE1.seconds()>timerInter)
                    {
                        timerCOLLECT_RAPID_FIRE2.reset();
                        closeClawController.CurrentStatus = CloseClawController.closeClawStatus.CLOSED;
                        CurrentStatus = COLLECT_RAPID_FIRE_INTER2_AUTO;
                    }
                    break;
                }
                case COLLECT_RAPID_FIRE_INTER2_AUTO:
                {
                    if (Cone_Stack_Level+1 !=5)
                    {
                        if (robotController.CurrentStatus == RobotController.RobotControllerStatus.START && timerCOLLECT_RAPID_FIRE2.seconds()>0.8)
                        {
                            robotController.CurrentStatus = RobotController.RobotControllerStatus.GO_PLACE;
                        }
                        if (timerCOLLECT_RAPID_FIRE2.seconds()>1.2)
                        {
                            motorColectareController.CurrentStatus = MotorColectareController.MotorColectare.RETRACTED;
                            timerLIFT.reset();
                            CurrentStatus = LIFT_CONE;
                        }
                    }
                    else
                    {
                        if (robotController.CurrentStatus == RobotController.RobotControllerStatus.START && timerCOLLECT_RAPID_FIRE2.seconds()>0.8)
                        {
                            robotController.CurrentStatus = RobotController.RobotControllerStatus.GO_PLACE;
                        }
                        if (motorColectareController.CurrentStatus == CLOSE_TO_EXTENDED_FIRST_CONE && timerCOLLECT_RAPID_FIRE2.seconds()>1.2)
                        {
                            motorColectareController.MotorColectarePID.maxOutput=0.55;
                            motorColectareController.CurrentStatus = EXTENDED_FIRST_CONE;
                        }
                        if (timerCOLLECT_RAPID_FIRE2.seconds()>1.5)
                        {
                            motorColectareController.MotorColectarePID.maxOutput=0.6;
                            motorColectareController.CurrentStatus = MotorColectareController.MotorColectare.RETRACTED;
                            timerLIFT.reset();
                            CurrentStatus = LIFT_CONE;
                        }
                    }
                    break;
                }
                case LIFT_CONE:
                {
                    if (timerLIFT.seconds()>1.5)
                    {
                        liftController.CurrentStatus = LiftController.LiftStatus.HIGH;

                        CurrentStatus = NOTHING;
                    }
                    break;
                }
                case STACK_LEVEL:
                {
                    if (liftController.CurrentStatus == LiftController.LiftStatus.HIGH)
                    {
                        liftController.CurrentStatus = LiftController.LiftStatus.BASE;
                        timeStart=1;
                    }
                    else timeStart=0;
                    if (Cone_Stack_Level==5)
                    {
                        servo4BarController.Collect_Position = servo4BarController.Fifth_Cone_Position;
                        Cone_Stack_Level =4;
                    }
                    else if (Cone_Stack_Level==4)
                    {
                        servo4BarController.Collect_Position = servo4BarController.Fourth_Cone_Position;
                        Cone_Stack_Level =3;
                    }
                    else if (Cone_Stack_Level==3)
                    {
                        servo4BarController.Collect_Position = servo4BarController.Third_Cone_Position;
                        Cone_Stack_Level =2;
                    }
                    else if (Cone_Stack_Level==2)
                    {
                        servo4BarController.Collect_Position = servo4BarController.Second_Cone_Position;
                        Cone_Stack_Level =1;
                    }
                    else if (Cone_Stack_Level==1)
                    {
                        servo4BarController.Collect_Position = servo4BarController.Ground_Position;
                        Cone_Stack_Level =5;
                    }
                    timerStart.reset();
                    CurrentStatus = COLLECT_RAPID_FIRE_AUTO;
                    break;
                }
            }
        }
        PreviousStatus = CurrentStatus;
    }
}
