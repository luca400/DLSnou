package org.firstinspires.ftc.teamcode.drive.OGCode;

import com.acmerobotics.dashboard.config.Config;

import org.checkerframework.checker.units.qual.Current;
@Config
public class Angle4BarController {

    public enum angle4BarStatus
    {
        INIT,
        VERTICAL,
        COLLECT_CONES,
        RAISED,
        PLACE,
        LIL_FRONT,
        LIL_RAISED,
        PLACE_LOW,
        LIL_PLACE
    }
    public static angle4BarStatus CurrentStatus = angle4BarStatus.INIT,  PreviousStatus = angle4BarStatus.INIT;
    public static double pozLilFront=0.45 , pozVertical = 0.47, pozCollectCones = 0.9 , pozRaised = 0.32 ,pozPlace = 0.54 , pozLilRaised = 0.34 , pozPlaceLow = 0.2,pozLilPlace = 0.5;

    public void update(RobotMap Robotel)
    {
            switch(CurrentStatus)
            {
                case VERTICAL:
                {
                    Robotel.angle4Bar.setPosition(pozVertical);
                    break;
                }
                case LIL_FRONT:
                {
                    Robotel.angle4Bar.setPosition(pozLilFront);
                    break;
                }
                case LIL_PLACE:
                {
                    Robotel.angle4Bar.setPosition(pozLilPlace);
                    break;
                }
                case COLLECT_CONES:
                {
                    Robotel.angle4Bar.setPosition(pozCollectCones);
                    break;
                }
                case RAISED:
                {
                    Robotel.angle4Bar.setPosition(pozRaised);
                    break;
                }
                case PLACE:
                {
                    Robotel.angle4Bar.setPosition(pozPlace);
                    break;
                }
                case LIL_RAISED:
                {
                    Robotel.angle4Bar.setPosition(pozLilRaised);
                    break;
                }
                case PLACE_LOW:
                {
                    Robotel.angle4Bar.setPosition(pozPlaceLow);
                    break;
                }

            }
        PreviousStatus = CurrentStatus;
    }
}
