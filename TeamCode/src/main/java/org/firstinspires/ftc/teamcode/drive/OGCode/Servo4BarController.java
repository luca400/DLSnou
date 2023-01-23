package org.firstinspires.ftc.teamcode.drive.OGCode;

import static org.firstinspires.ftc.teamcode.drive.OGCode.Servo4BarController.ServoStatus.COLLECT_DRIVE;
import static org.firstinspires.ftc.teamcode.drive.OGCode.Servo4BarController.ServoStatus.INITIALIZE;
import static org.firstinspires.ftc.teamcode.drive.OGCode.Servo4BarController.ServoStatus.INTERMEDIARY;
import static org.firstinspires.ftc.teamcode.drive.OGCode.Servo4BarController.ServoStatus.PLACE_CONE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Servo4BarController {
    public enum ServoStatus
    {
        INITIALIZE,
        COLLECT_DRIVE,
        INTERMEDIARY,
        PLACE_CONE,
        DRIVE_POSITION,
        STACK_POSITION
    }
    public static ServoStatus CurrentStatus = INITIALIZE,PreviousStatus = INITIALIZE,WhereFromIntermediary = COLLECT_DRIVE;
    ElapsedTime time = new ElapsedTime();
    public static double Ground_Position=0.09, Second_Cone_Position=0.13, Third_Cone_Position=0.17, Fourth_Cone_Position = 0.21, Fifth_Cone_Position = 0.26;
    public static double Collect_Position = Ground_Position , Place_Cone_Position = 0.8 , Intermediary_Position =0.4 , Drive_Position = 0.6;
int salut =0;
    public void update(RobotMap Robot)
    {
        if (PreviousStatus != CurrentStatus || CurrentStatus==INTERMEDIARY || CurrentStatus==INITIALIZE)
        {
            switch(CurrentStatus)
            {
                case INITIALIZE:
                {
                    Robot.left4Bar.setPosition(Drive_Position);
                    Robot.right4Bar.setPosition(Drive_Position);
                    break;
                }
                case INTERMEDIARY:
                {
                    salut = 2;
                    if (WhereFromIntermediary == COLLECT_DRIVE)
                    {
                        if (time.seconds()> 0.3)
                        {

                            Robot.right4Bar.setPosition(Place_Cone_Position);
                            Robot.left4Bar.setPosition(Place_Cone_Position);
                            CurrentStatus = PLACE_CONE;
                        }
                    }
                    else {
                        if (time.seconds() > 0.3) {
                            Robot.right4Bar.setPosition(Collect_Position);
                            Robot.left4Bar.setPosition(Collect_Position);
                            CurrentStatus = COLLECT_DRIVE;
                        }
                    }
                    break;
                }
                case COLLECT_DRIVE:
                {
                    if (PreviousStatus == PLACE_CONE)
                    {
                        time.reset();
                        Robot.left4Bar.setPosition(Intermediary_Position);
                        Robot.right4Bar.setPosition(Intermediary_Position);
                        WhereFromIntermediary = PLACE_CONE;
                        CurrentStatus = INTERMEDIARY;
                    }
                    else if (PreviousStatus == INITIALIZE)
                    {
                        time.reset();
                        Robot.left4Bar.setPosition(Collect_Position);
                        Robot.right4Bar.setPosition(Collect_Position);
                    }
                    break;
                }
                case PLACE_CONE:
                {
                    time.reset();
                    if (PreviousStatus == COLLECT_DRIVE)
                    {
                        salut=1;
                        Robot.left4Bar.setPosition(Intermediary_Position);
                        Robot.right4Bar.setPosition(Intermediary_Position);
                        WhereFromIntermediary = COLLECT_DRIVE;
                        CurrentStatus = INTERMEDIARY;
                    }
                    break;
                }
                case DRIVE_POSITION:
                {
                    Robot.left4Bar.setPosition(Drive_Position);
                    Robot.right4Bar.setPosition(Drive_Position);
                    CurrentStatus = PLACE_CONE;
                    break;
                }
                case STACK_POSITION:
                {
                    CurrentStatus = COLLECT_DRIVE;
                    break;
                }
            }
        }
        PreviousStatus = CurrentStatus;
    }

}
