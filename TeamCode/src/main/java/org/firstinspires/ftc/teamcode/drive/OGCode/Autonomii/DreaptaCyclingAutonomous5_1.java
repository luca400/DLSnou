package org.firstinspires.ftc.teamcode.drive.OGCode.Autonomii;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.OGCode.AutoController;
import org.firstinspires.ftc.teamcode.drive.OGCode.BiggerController;
import org.firstinspires.ftc.teamcode.drive.OGCode.CloseClawController;
import org.firstinspires.ftc.teamcode.drive.OGCode.LiftController;
import org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController;
import org.firstinspires.ftc.teamcode.drive.OGCode.PipeLineDetector;
import org.firstinspires.ftc.teamcode.drive.OGCode.RobotController;
import org.firstinspires.ftc.teamcode.drive.OGCode.RobotMap;
import org.firstinspires.ftc.teamcode.drive.OGCode.Servo4BarController;
import org.firstinspires.ftc.teamcode.drive.OGCode.ServoLiftController;
import org.firstinspires.ftc.teamcode.drive.OGCode.SigurantaLiftController;
import org.firstinspires.ftc.teamcode.drive.OGCode.TurnClawController;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;


@Config
@Autonomous(group = "drive")

public class DreaptaCyclingAutonomous5_1 extends LinearOpMode {
    enum STROBOT
    {
        START,
        PLACE,
        GO_DOWN,
        GO_TO_STACK,
        STOP_JOC,
        PARK,
        GET_LIFT_DOWN,
        FOURTH_CONE,
        START_CYCLING,
        TURN_TO_PLACE,
        GET_LIFT_DOWN_WITH_TURN,
        TURN_TO_CYCLING_POSITION,
    }


    public static double x_PLACE_PRELOAD = 35, y_PLACE_PRELOAD = -10, Angle_PLACE_PRELOAD=315;
    public static double x_GO_TO_STACK_FIRST = 54, y_GO_TO_STACK_FIRST = -13 , Angle_GO_TO_STACK_FIRST = 0;
    public static double x_CYCLING_POSITION = 37.65, y_CYCLING_POSITION = -13 , Angle_CYCLING_POSITION = 2.5;
    ElapsedTime asteapta = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        RobotMap robot = new RobotMap(hardwareMap);
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);



        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        Servo4BarController servo4BarController = new Servo4BarController();
        MotorColectareController motorColectareController = new MotorColectareController();
        CloseClawController closeClawController = new CloseClawController();
        TurnClawController turnClawController = new TurnClawController();
        RobotController robotController = new RobotController();
        BiggerController biggerController = new BiggerController();
        LiftController liftController = new LiftController();
        ServoLiftController servoLiftController = new ServoLiftController();
        AutoController autoController = new AutoController();
        SigurantaLiftController sigurantaLiftController = new SigurantaLiftController();

        servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.INITIALIZE;
        motorColectareController.CurrentStatus = MotorColectareController.MotorColectare.RETRACTED;
        closeClawController.CurrentStatus = CloseClawController.closeClawStatus.CLOSED;
        turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.COLLECT;
        robotController.CurrentStatus = RobotController.RobotControllerStatus.START;
        liftController.CurrentStatus = LiftController.LiftStatus.BASE;
        servoLiftController.CurrentStatus = ServoLiftController.ServoLiftStatus.TRANSFER;
        sigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.TRANSFER;

        closeClawController.update(robot);
        turnClawController.update(robot);
        servo4BarController.update(robot);
        servoLiftController.update(robot,sigurantaLiftController);
        sigurantaLiftController.update(robot);
        motorColectareController.update(robot,0, 0.6);
        liftController.update(robot,0,sigurantaLiftController,servoLiftController);
        robotController.update(servoLiftController,servo4BarController,motorColectareController,closeClawController,turnClawController);
        biggerController.update(robotController,closeClawController,motorColectareController);
        int nr=0;
        Pose2d startPose = new Pose2d(35, -63, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        STROBOT status = STROBOT.START;
        TrajectorySequence PLACE_PRELOAD = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(x_PLACE_PRELOAD,y_PLACE_PRELOAD,Math.toRadians(Angle_PLACE_PRELOAD)))
                .addTemporalMarker(1.5, ()-> {
                    liftController.CurrentStatus = LiftController.LiftStatus.HIGH;
                })
                .addTemporalMarker(0,()->{
                    servoLiftController.CurrentStatus = ServoLiftController.ServoLiftStatus.JUNCTION;
                })
                .build();
        TrajectorySequence GO_TO_STACK = drive.trajectorySequenceBuilder(PLACE_PRELOAD.end())
                .lineToLinearHeading(new Pose2d(x_GO_TO_STACK_FIRST,y_GO_TO_STACK_FIRST, Angle_GO_TO_STACK_FIRST))
                .addTemporalMarker(3, ()-> {
                    autoController.CurrentStatus = AutoController.autoControllerStatus.STACK_LEVEL;
                })
                .build();
        TrajectorySequence PLACE_STACK = drive.trajectorySequenceBuilder(GO_TO_STACK.end())
                .lineToLinearHeading(new Pose2d(x_PLACE_PRELOAD,y_PLACE_PRELOAD,Math.toRadians(Angle_PLACE_PRELOAD)))
                .addTemporalMarker(0,()->{
                    liftController.CurrentStatus = LiftController.LiftStatus.HIGH;
                })
                .build();
        TrajectorySequence CYCLING_POSITION = drive.trajectorySequenceBuilder(PLACE_STACK.end())
                .lineToLinearHeading(new Pose2d(x_CYCLING_POSITION,y_CYCLING_POSITION,Math.toRadians(Angle_CYCLING_POSITION)))
                .build();
        TrajectorySequence TURN_TO_PLACE = drive.trajectorySequenceBuilder(CYCLING_POSITION.end())
                .turn(Math.toRadians(-40))
                .build();
        TrajectorySequence TURN_TO_CYCLING_POSITION = drive.trajectorySequenceBuilder(TURN_TO_PLACE.end())
                .turn(Math.toRadians(40))
                .build();
        TrajectorySequence PARK_3 = drive.trajectorySequenceBuilder(CYCLING_POSITION.end())
                .lineToLinearHeading(new Pose2d(x_CYCLING_POSITION+20,y_CYCLING_POSITION-5,Math.toRadians(90)))
                .build();
        TrajectorySequence PARK_1 = drive.trajectorySequenceBuilder(CYCLING_POSITION.end())
                .lineToLinearHeading(new Pose2d(x_CYCLING_POSITION-25,y_CYCLING_POSITION-5,Math.toRadians(90)))
                .build();
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        PipeLineDetector detector = new PipeLineDetector(270,195,320,225);
        camera.setPipeline(detector);
        camera.openCameraDeviceAsync(
                new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        camera.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
                    }

                    @Override
                    public void onError(int errorCode) {

                    }
                }
        );
        PipeLineDetector.Status Case = PipeLineDetector.Status.ALBASTRU3;
        while (!isStarted()&&!isStopRequested())
        {
            Case = detector.caz;
            telemetry.addData("Caz", detector.caz);
            telemetry.addLine("Init Complete");
            telemetry.update();
            sleep(50);
        }
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested())
        {
            int ColectarePosition = robot.encoderMotorColectare.getCurrentPosition();
            int LiftPosition = robot.dreaptaLift.getCurrentPosition(); /// folosesc doar encoderul de la dreaptaLift , celalalt nu exista
            if (status == STROBOT.START)
            {
                drive.followTrajectorySequenceAsync(PLACE_PRELOAD);
                asteapta.reset();
                status = STROBOT.GO_DOWN;
            }
            else
            if (status == STROBOT.GO_DOWN)
            {
                if (!drive.isBusy() && asteapta.seconds()> 3.5)
                {
                    liftController.CurrentStatus = LiftController.LiftStatus.BASE;
                    status = STROBOT.GO_TO_STACK;
                    asteapta.reset();
                }
            }
            else
            if (status == STROBOT.GO_TO_STACK)
            {
                if (asteapta.seconds()>1) {
                    drive.followTrajectorySequenceAsync(GO_TO_STACK);
                    asteapta.reset();
                    status = STROBOT.PLACE;
                }
            }
            else
            if (status == STROBOT.PLACE)
            {
                if (!drive.isBusy() && asteapta.seconds()>6) {
                    drive.followTrajectorySequenceAsync(PLACE_STACK);
                    asteapta.reset();
                    status = STROBOT.GET_LIFT_DOWN;
                }
            }
            else
            if (status == STROBOT.GET_LIFT_DOWN)
            {
                if (!drive.isBusy() && asteapta.seconds()>2)
                {
                    liftController.CurrentStatus = LiftController.LiftStatus.BASE;
                    status = STROBOT.FOURTH_CONE;
                }
            }
            else
            if (status == STROBOT.FOURTH_CONE)
            {
                if (LiftPosition == 0)
                {
                    drive.followTrajectorySequenceAsync(CYCLING_POSITION);
                    status = STROBOT.START_CYCLING;
                }
            }
            else
            if (status == STROBOT.START_CYCLING)
            {
                if (!drive.isBusy())
                {
                    nr++;
                    if (nr<=2)
                    {
                        autoController.CurrentStatus = AutoController.autoControllerStatus.STACK_LEVEL;
                        status = STROBOT.TURN_TO_PLACE;
                    }
                    else
                    {
                        if (detector.caz == PipeLineDetector.Status.ALBASTRU3)
                        {
                            drive.followTrajectorySequenceAsync(PARK_3);
                        }
                        else
                        if (detector.caz == PipeLineDetector.Status.VERDE1)
                        {
                            drive.followTrajectorySequenceAsync(PARK_1);
                        }
                        status = STROBOT.PARK;
                    }
                }
            }
            else
            if (status == STROBOT.TURN_TO_PLACE)
            {
                if (autoController.CurrentStatus == AutoController.autoControllerStatus.LIFT_CONE && autoController.timerLIFT.seconds()>0.1)
                {
                    asteapta.reset();
                    drive.followTrajectorySequenceAsync(TURN_TO_PLACE);
                    status = STROBOT.GET_LIFT_DOWN_WITH_TURN;
                }
            }
            else
            if (status == STROBOT.GET_LIFT_DOWN_WITH_TURN)
            {
                if (!drive.isBusy() && asteapta.seconds()>2.5)
                {
                    liftController.CurrentStatus = LiftController.LiftStatus.BASE;
                    status = STROBOT.TURN_TO_CYCLING_POSITION;
                }
            }
            else
            if (status == STROBOT.TURN_TO_CYCLING_POSITION)
            {
                if (LiftPosition < 1600)
                {
                    drive.followTrajectorySequenceAsync(TURN_TO_CYCLING_POSITION);
                    status = STROBOT.START_CYCLING;
                }
            }
            biggerController.update(robotController,closeClawController,motorColectareController);
            robotController.update(servoLiftController,servo4BarController,motorColectareController,closeClawController,turnClawController);
            closeClawController.update(robot);
            turnClawController.update(robot);
            servo4BarController.update(robot);
            servoLiftController.update(robot,sigurantaLiftController);
            sigurantaLiftController.update(robot);
            motorColectareController.update(robot,ColectarePosition, 0.6);
            liftController.update(robot,LiftPosition,sigurantaLiftController,servoLiftController);
            autoController.update(robot,turnClawController, servoLiftController, liftController, servo4BarController, robotController, closeClawController, motorColectareController);

            drive.update();
            telemetry.addData("Pozitie: ", drive.getPoseEstimate());
           // telemetry.addData("caz:", Case);
            telemetry.addData("Status",status);
            telemetry.update();
        }
    }

}