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
    double Place_Cone_Position = 0 , Collect_Drive_Position = 0.85 , Intermediary_Position =0.4;
    double pozOpenClaw = 0.2, pozCloseClaw = 0.8;
    double pozTurnClaw1 = 0.7, pozTurnClaw0 = 0;

    void update(RobotMap Robot)
    {
        if (PreviousStatus != CurrentStatus||CurrentStatus==INTERMEDIARY || CurrentStatus==INITIALIZE)
        {
            SampleOpModeDLS.salut = 1 ;
            switch(CurrentStatus)
            {
                case INITIALIZE:
                {
                    Robot.left4Bar.setPosition(0.4);
                    Robot.right4Bar.setPosition(0.4);
                    Robot.closeClaw.setPosition(pozOpenClaw);
                    Robot.turnCLaw.setPosition(pozTurnClaw1);
                    Robot.motorColectare.setTargetPosition(0);
                    Robot.motorColectare.setPower(0.6);
                    Robot.motorColectare.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    break;
                }
                case INTERMEDIARY:
                {
                    if (WhereFromIntermediary == COLLECT_DRIVE)
                    {
                        if (time.seconds()>0.5){
                        Robot.right4Bar.setPosition(Place_Cone_Position);
                        Robot.left4Bar.setPosition(Place_Cone_Position);
                        }
                        if (time.seconds()>1.2)
                        {
                            Robot.closeClaw.setPosition(pozOpenClaw);
                            CurrentStatus= PLACE_CONE;
                            break;
                        }

                    }
                    else {
                        if (time.seconds() > 0.5) {
                            Robot.motorColectare.setTargetPosition(250);
                            Robot.motorColectare.setPower(0.6);
                            Robot.motorColectare.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        }
                        if (time.seconds() > 1) {
                            Robot.right4Bar.setPosition(Collect_Drive_Position);
                            Robot.left4Bar.setPosition(Collect_Drive_Position);
                            Robot.closeClaw.setPosition(pozOpenClaw);
                            Robot.turnCLaw.setPosition(pozTurnClaw1);
                            CurrentStatus = COLLECT_DRIVE;
                            break;
                        }
                    }



                }
                case COLLECT_DRIVE:
                {
                    if (PreviousStatus == PLACE_CONE)
                    {
                        time.reset();
                        Robot.left4Bar.setPosition(Intermediary_Position);
                        Robot.right4Bar.setPosition(Intermediary_Position);
                        Robot.closeClaw.setPosition(pozCloseClaw);
                        WhereFromIntermediary = PLACE_CONE;
                        CurrentStatus = INTERMEDIARY;
                    }
                    else if (PreviousStatus == INITIALIZE)
                    {
                        time.reset();
                        Robot.left4Bar.setPosition(Collect_Drive_Position);
                        Robot.right4Bar.setPosition(Collect_Drive_Position);
                        Robot.motorColectare.setTargetPosition(250);
                        Robot.motorColectare.setPower(0.6);
                        Robot.motorColectare.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
                        Robot.turnCLaw.setPosition(pozTurnClaw0);
                        Robot.motorColectare.setTargetPosition(0);
                        Robot.motorColectare.setPower(0.6);
                        Robot.motorColectare.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
