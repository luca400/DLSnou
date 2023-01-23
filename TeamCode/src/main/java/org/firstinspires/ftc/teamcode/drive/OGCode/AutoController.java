package org.firstinspires.ftc.teamcode.drive.OGCode;

import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoController.autoControllerStatus.COLLECT_RAPID_FIRE_AUTO;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoController.autoControllerStatus.COLLECT_RAPID_FIRE_FIRST_CONE;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoController.autoControllerStatus.COLLECT_RAPID_FIRE_FIRST_CONE_2;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoController.autoControllerStatus.COLLECT_RAPID_FIRE_FIRST_CONE_3;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoController.autoControllerStatus.COLLECT_RAPID_FIRE_FIRST_CONE_4;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoController.autoControllerStatus.COLLECT_RAPID_FIRE_FIRST_CONE_5;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoController.autoControllerStatus.COLLECT_RAPID_FIRE_FIRST_CONE_6;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoController.autoControllerStatus.COLLECT_RAPID_FIRE_INTER_AUTO;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoController.autoControllerStatus.COLLECT_RAPID_FIRE_INTER2_AUTO;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoController.autoControllerStatus.LIFT_CONE;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoController.autoControllerStatus.NOTHING;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.CLOSE_TO_EXTENDED_FIRST_CONE;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.EXTENDED;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.EXTENDED_600;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.EXTENDED_FIRST_CONE;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.RETRACTED;

import com.qualcomm.robotcore.util.ElapsedTime;

public class AutoController {
    public enum autoControllerStatus
    {
        NOTHING,
        COLLECT_RAPID_FIRE_AUTO,
        COLLECT_RAPID_FIRE_INTER_AUTO,
        COLLECT_RAPID_FIRE_INTER2_AUTO,
        COLLECT_RAPID_FIRE_FIRST_CONE,
        COLLECT_RAPID_FIRE_FIRST_CONE_2,
        COLLECT_RAPID_FIRE_FIRST_CONE_3,
        COLLECT_RAPID_FIRE_FIRST_CONE_4,
        COLLECT_RAPID_FIRE_FIRST_CONE_5,
        COLLECT_RAPID_FIRE_FIRST_CONE_6,
        STACK_LEVEL,
        LIFT_CONE,
        INITIALIZE
    }
    public static autoControllerStatus CurrentStatus = NOTHING, PreviousStatus = NOTHING;
    public ElapsedTime timerCOLLECT_RAPID_FIRE2 = new ElapsedTime() ,timerCOLLECT_RAPID_FIRE1 = new ElapsedTime(),
            timerLIFT = new ElapsedTime(), timerStart = new ElapsedTime() , timerCOLLECT_RAPID_FIRE_FIRST_CONE = new ElapsedTime();
    ElapsedTime timerCOLLECT_RAPID_FIRE_FIRST_CONE_2 = new ElapsedTime() , timerCOLLECT_RAPID_FIRE_FIRST_CONE_3 = new ElapsedTime() ,timerCOLLECT_RAPID_FIRE_FIRST_CONE_4 = new ElapsedTime(),timerCOLLECT_RAPID_FIRE_FIRST_CONE_5 = new ElapsedTime(), timerCOLLECT_RAPID_FIRE_FIRST_CONE_6 = new ElapsedTime();
    int Cone_Stack_Level=5;
    double timerInter = 2,timeStart=0;
    public void update(RobotMap Robotel,TurnClawController turnClawController, ServoLiftController servoLiftController, LiftController liftController, Servo4BarController servo4BarController, RobotController robotController, CloseClawController closeClawController, MotorColectareController motorColectareController)
    {
        if (CurrentStatus!=PreviousStatus || CurrentStatus == COLLECT_RAPID_FIRE_FIRST_CONE_2  || CurrentStatus == COLLECT_RAPID_FIRE_FIRST_CONE_3 || CurrentStatus == COLLECT_RAPID_FIRE_FIRST_CONE_4 || CurrentStatus == COLLECT_RAPID_FIRE_FIRST_CONE_5 || CurrentStatus == COLLECT_RAPID_FIRE_FIRST_CONE_6 || CurrentStatus == COLLECT_RAPID_FIRE_FIRST_CONE || CurrentStatus == LIFT_CONE || CurrentStatus == COLLECT_RAPID_FIRE_AUTO ||CurrentStatus ==  COLLECT_RAPID_FIRE_INTER_AUTO|| CurrentStatus == COLLECT_RAPID_FIRE_INTER2_AUTO)
        {
            switch (CurrentStatus)
            {
                case INITIALIZE:
                {
                    servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.INITIALIZE;
                    motorColectareController.CurrentStatus = RETRACTED;
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
                        timerCOLLECT_RAPID_FIRE_FIRST_CONE.reset();
                        robotController.CurrentStatus = RobotController.RobotControllerStatus.GO_COLLECT;
                        CurrentStatus = COLLECT_RAPID_FIRE_FIRST_CONE;
                    }
                    else
                    {
                        if (robotController.CurrentStatus == RobotController.RobotControllerStatus.START&&timerStart.seconds()>timeStart)
                        {
                            robotController.CurrentStatus = RobotController.RobotControllerStatus.GO_COLLECT;
                        }
                        if (timerStart.seconds()>timeStart+0.5)
                        {
                            timerCOLLECT_RAPID_FIRE1.reset();
                            motorColectareController.CurrentStatus = EXTENDED;
                            CurrentStatus = COLLECT_RAPID_FIRE_INTER_AUTO;
                        }
                    }
                    break;
                }
                case COLLECT_RAPID_FIRE_FIRST_CONE:
                {
                    if (timerCOLLECT_RAPID_FIRE_FIRST_CONE.seconds()>1)
                    {
                        timerCOLLECT_RAPID_FIRE_FIRST_CONE_2.reset();
                        closeClawController.CurrentStatus = CloseClawController.closeClawStatus.CLOSED;
                        CurrentStatus = COLLECT_RAPID_FIRE_FIRST_CONE_2;
                    }
                    break;
                }
                case COLLECT_RAPID_FIRE_FIRST_CONE_2:
                {
                    if (timerCOLLECT_RAPID_FIRE_FIRST_CONE_2.seconds()>0.2&&motorColectareController.CurrentStatus != EXTENDED_600)
                    {
                        motorColectareController.CurrentStatus = EXTENDED_600;
                    }
                    if (timerCOLLECT_RAPID_FIRE_FIRST_CONE_2.seconds()>0.5)
                    {
                        servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.DRIVE_POSITION;
                        timerCOLLECT_RAPID_FIRE_FIRST_CONE_3.reset();
                        CurrentStatus = COLLECT_RAPID_FIRE_FIRST_CONE_3;
                    }
                    break;
                }
                case COLLECT_RAPID_FIRE_FIRST_CONE_3:
                {
                    if (turnClawController.CurrentStatus != TurnClawController.TurnClawStatus.PLACE && timerCOLLECT_RAPID_FIRE_FIRST_CONE_3.seconds() >1)
                    {
                        turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.PLACE;
                    }
                    if (timerCOLLECT_RAPID_FIRE_FIRST_CONE_3.seconds()>1.5)
                    {
                        motorColectareController.CurrentStatus = RETRACTED;
                        timerCOLLECT_RAPID_FIRE_FIRST_CONE_4.reset();
                        CurrentStatus = COLLECT_RAPID_FIRE_FIRST_CONE_4;
                    }
                    break;
                }
                case COLLECT_RAPID_FIRE_FIRST_CONE_4:
                {
                    if (timerCOLLECT_RAPID_FIRE_FIRST_CONE_4.seconds()>0.3)
                    {
                        timerCOLLECT_RAPID_FIRE_FIRST_CONE_5.reset();
                        Robotel.left4Bar.setPosition(servo4BarController.Place_Cone_Position);
                        Robotel.right4Bar.setPosition(servo4BarController.Place_Cone_Position);
                        CurrentStatus = COLLECT_RAPID_FIRE_FIRST_CONE_5;
                    }
                    break;
                }
                case COLLECT_RAPID_FIRE_FIRST_CONE_5:
                {
                    if (timerCOLLECT_RAPID_FIRE_FIRST_CONE_5.seconds()>0.3)
                    {
                        timerCOLLECT_RAPID_FIRE_FIRST_CONE_6.reset();
                        closeClawController.CurrentStatus = CloseClawController.closeClawStatus.OPEN;
                        CurrentStatus = COLLECT_RAPID_FIRE_FIRST_CONE_6;
                    }
                    break;
                }
                case COLLECT_RAPID_FIRE_FIRST_CONE_6:
                {
                    if (timerCOLLECT_RAPID_FIRE_FIRST_CONE_6.seconds()>0.3)
                    {
                        servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.DRIVE_POSITION;
                        timerLIFT.reset();
                        CurrentStatus = NOTHING;
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
                        if (robotController.CurrentStatus == RobotController.RobotControllerStatus.START && timerCOLLECT_RAPID_FIRE2.seconds()>0.8)
                        {
                            robotController.CurrentStatus = RobotController.RobotControllerStatus.GO_PLACE;
                        }
                        if (timerCOLLECT_RAPID_FIRE2.seconds()>1.2)
                        {
                            motorColectareController.CurrentStatus = RETRACTED;
                            timerLIFT.reset();
                            CurrentStatus = LIFT_CONE;
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
                    robotController.timerTransfer = 1.5;
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
