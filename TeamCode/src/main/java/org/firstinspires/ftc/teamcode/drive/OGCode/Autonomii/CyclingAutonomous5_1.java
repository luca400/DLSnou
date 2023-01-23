package org.firstinspires.ftc.teamcode.drive.OGCode.Autonomii;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.OGCode.AutoController;
import org.firstinspires.ftc.teamcode.drive.OGCode.BiggerController;
import org.firstinspires.ftc.teamcode.drive.OGCode.CloseClawController;
import org.firstinspires.ftc.teamcode.drive.OGCode.LiftController;
import org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController;
import org.firstinspires.ftc.teamcode.drive.OGCode.RobotController;
import org.firstinspires.ftc.teamcode.drive.OGCode.RobotMap;
import org.firstinspires.ftc.teamcode.drive.OGCode.Servo4BarController;
import org.firstinspires.ftc.teamcode.drive.OGCode.ServoLiftController;
import org.firstinspires.ftc.teamcode.drive.OGCode.SigurantaLiftController;
import org.firstinspires.ftc.teamcode.drive.OGCode.TurnClawController;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;


@Config
@Autonomous(group = "drive")

public class CyclingAutonomous5_1 extends LinearOpMode {
    enum STROBOT
    {
        START,
        PLACE,
        GO_DOWN,
        GO_TO_STACK,
        STOP_JOC

    }


    public static double x_PLACE_PRELOAD = 35, y_PLACE_PRELOAD = -10, Angle_PLACE_PRELOAD=315;
    public static double x_GO_TO_STACK_FIRST = 37, y_GO_TO_STACK_FIRST = -13 , Angle_GO_TO_STACK_FIRST = 0;

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
        servoLiftController.update(robot);
        sigurantaLiftController.update(robot);
        motorColectareController.update(robot,0, 0.6);
        liftController.update(robot,0,sigurantaLiftController,servoLiftController);
        robotController.update(servoLiftController,servo4BarController,motorColectareController,closeClawController,turnClawController);
        biggerController.update(robotController,closeClawController,motorColectareController);

        Pose2d startPose = new Pose2d(35, -63, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        STROBOT status = STROBOT.START;
        TrajectorySequence PLACE_PRELOAD = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(x_PLACE_PRELOAD,y_PLACE_PRELOAD,Math.toRadians(Angle_PLACE_PRELOAD)))
                .addTemporalMarker(0.5, ()-> {
                    liftController.CurrentStatus = LiftController.LiftStatus.HIGH;
                })
                .addTemporalMarker(0,()->{
                    servoLiftController.CurrentStatus = ServoLiftController.ServoLiftStatus.JUNCTION;
                })
                .build();
        TrajectorySequence GO_TO_STACK = drive.trajectorySequenceBuilder(PLACE_PRELOAD.end())
                .lineToLinearHeading(new Pose2d(x_GO_TO_STACK_FIRST,y_GO_TO_STACK_FIRST, Angle_GO_TO_STACK_FIRST))
                .addTemporalMarker(0, ()-> {
                    autoController.CurrentStatus = AutoController.autoControllerStatus.STACK_LEVEL;
                })
                .build();
        TrajectorySequence PLACE_STACK = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(x_PLACE_PRELOAD,y_PLACE_PRELOAD,Math.toRadians(Angle_PLACE_PRELOAD)))
                .build();




        /*int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);*/
        /*PipeLineDetector detector = new PipeLineDetector(340,125,390,195);
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
        }*/
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested())
        {
            if (status == STROBOT.START)
            {
                drive.followTrajectorySequenceAsync(PLACE_PRELOAD);
                status = STROBOT.GO_DOWN;
            }
            else
            if (status == STROBOT.GO_DOWN)
            {
                if (!drive.isBusy())
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
                    status = STROBOT.STOP_JOC;
                }
            }
            if (status == STROBOT.PLACE)
            {
                if (!drive.isBusy()&&asteapta.seconds()>3) {
                    //drive.followTrajectorySequenceAsync(PLACE_STACK);
                    status = STROBOT.STOP_JOC;
                }
            }
            int ColectarePosition = robot.encoderMotorColectare.getCurrentPosition();
            int LiftPosition = robot.dreaptaLift.getCurrentPosition(); /// folosesc doar encoderul de la dreaptaLift , celalalt nu exista.

            biggerController.update(robotController,closeClawController,motorColectareController);
            robotController.update(servoLiftController,servo4BarController,motorColectareController,closeClawController,turnClawController);
            closeClawController.update(robot);
            turnClawController.update(robot);
            servo4BarController.update(robot);
            servoLiftController.update(robot);
            sigurantaLiftController.update(robot);
            motorColectareController.update(robot,ColectarePosition, 0.6);
            liftController.update(robot,LiftPosition,sigurantaLiftController,servoLiftController);
            autoController.update(turnClawController, servoLiftController, liftController, servo4BarController, robotController, closeClawController, motorColectareController);

            drive.update();
            telemetry.addData("Pozitie: ", drive.getPoseEstimate());
           // telemetry.addData("caz:", Case);
            telemetry.addData("Status",status);
            telemetry.update();
        }
    }

}