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
    enum ServoStatus
    {
        INITIALIZE,
        COLLECT_DRIVE,
        INTERMEDIARY,
        PLACE_CONE,

    }
    ServoStatus CurrentStatus = INITIALIZE,PreviousStatus = INITIALIZE,WhereFromIntermediary = INITIALIZE;
    ElapsedTime time = new ElapsedTime();
    double Collect_Drive_Position = 0.1 , Place_Cone_Position = 0.8 , Intermediary_Position =0.4;

    void update(RobotMap Robot)
    {
        if (PreviousStatus != CurrentStatus || CurrentStatus==INTERMEDIARY || CurrentStatus==INITIALIZE)
        {
            switch(CurrentStatus)
            {
                case INITIALIZE:
                {
                    Robot.left4Bar.setPosition(Intermediary_Position);
                    Robot.right4Bar.setPosition(Intermediary_Position);
                    break;
                }
                case INTERMEDIARY:
                {
                    if (WhereFromIntermediary == COLLECT_DRIVE)
                    {
                        if (time.seconds()> 0.5)
                        {
                            Robot.right4Bar.setPosition(Place_Cone_Position);
                            Robot.left4Bar.setPosition(Place_Cone_Position);
                            CurrentStatus = PLACE_CONE;
                        }
                    }
                    else {
                        if (time.seconds() > 0.5) {
                            Robot.right4Bar.setPosition(Collect_Drive_Position);
                            Robot.left4Bar.setPosition(Collect_Drive_Position);
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
                        Robot.left4Bar.setPosition(Collect_Drive_Position);
                        Robot.right4Bar.setPosition(Collect_Drive_Position);
                    }
                    break;
                }
                case PLACE_CONE:
                {
                    time.reset();
                    if (PreviousStatus == COLLECT_DRIVE)
                    {

                        Robot.left4Bar.setPosition(Intermediary_Position);
                        Robot.right4Bar.setPosition(Intermediary_Position);
                        WhereFromIntermediary = COLLECT_DRIVE;
                        CurrentStatus = INTERMEDIARY;
                        break;
                    }
                }
            }
        }
        PreviousStatus = CurrentStatus;
    }

}
