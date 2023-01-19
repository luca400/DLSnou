package org.firstinspires.ftc.teamcode.drive.OGCode;

import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.INITIALIZED;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.RETRACTED;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MotorColectareController {
    enum MotorColectare
    {
        INITIALIZED,
        EXTENDED,
        RETRACTED,
    }
    MotorColectare CurrentStatus = INITIALIZED,  PreviousStatus = INITIALIZED;
    int extendedPosition = 250 , retractedPosition = 0;
    void update(RobotMap Robotel)
    {
        if (PreviousStatus != CurrentStatus)
        {
            switch (CurrentStatus)
            {
                case RETRACTED:
                {
                    Robotel.motorColectare.setTargetPosition(0);
                    Robotel.motorColectare.setPower(0.6);
                    Robotel.motorColectare.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    break;
                }
                case EXTENDED:
                {
                    Robotel.motorColectare.setTargetPosition(extendedPosition);
                    Robotel.motorColectare.setPower(0.6);
                    Robotel.motorColectare.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            }
        }
        PreviousStatus = CurrentStatus;
    }
}
