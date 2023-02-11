package org.firstinspires.ftc.teamcode.drive.OGCode;


import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController5_1.autoControllerStatus.NOTHING;
import static org.firstinspires.ftc.teamcode.drive.OGCode.Servo4BarController.ServoStatus.STACK_POSITION;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController5_1;

import java.util.List;

@TeleOp(name="SampleOpModeDLS", group="Linear Opmode")

public class SampleOpModeDLS extends  LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime() , timeGetVoltage = new ElapsedTime();
    public static int salut=0;
    double pozInit4Bar = 0, pozInter4Bar= 0.4, pozPlace4Bar = 0.7;
    double pozCloseClaw=0.8, pozOpenClaw=0.2;
    double kp =0, ki=0, kd=0;
    boolean isDown = true, isClosed=false, isTurned = false, isExtended = false;
    double  PrecisionDenominator=1, PrecisionDenominator2=1.25;

    public void robotCentricDrive(DcMotor leftFront,DcMotor leftBack,DcMotor rightFront,DcMotor rightBack, double  lim, boolean StrafesOn)
    {
        double y = gamepad1.right_stick_y; // Remember, this is reversed!
        double x = gamepad1.right_stick_x*1.1;
        if (StrafesOn == false)
        {
            x=0;
        }
        double rx = gamepad1.left_stick_x*1;

        rx/=PrecisionDenominator2;
        x/=PrecisionDenominator;
        y/=PrecisionDenominator;
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftPower = Clip(frontLeftPower,lim);
        backLeftPower = Clip(backLeftPower,lim);
        frontRightPower = Clip(frontRightPower,lim);
        backRightPower = Clip(backRightPower,lim);

        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);
    }
    public void fieldCentricDrive(BNO055IMU imu,DcMotor leftFront, DcMotor leftBack, DcMotor rightFront, DcMotor rightBack )
    {
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Read inverse IMU heading, as the IMU heading is CW positive
        double botHeading = -imu.getAngularOrientation().firstAngle;

        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);
    }
    public void verifyYClaw()
    {

    }
    double Clip(double Speed,double lim)
    {
        return Math.max(Math.min(Speed,lim),-lim);
    }
    @Override
    public void runOpMode() {

        RobotMap robot=new RobotMap(hardwareMap);
        SigurantaLiftController sigurantaLiftController = new SigurantaLiftController();
        Angle4BarController angle4BarController = new Angle4BarController();
        Servo4BarController servo4BarController = new Servo4BarController();
        MotorColectareController motorColectareController = new MotorColectareController();
        CloseClawController closeClawController = new CloseClawController();
        TurnClawController turnClawController = new TurnClawController();
        RobotController robotController = new RobotController();
        BiggerController biggerController = new BiggerController();
        LiftController liftController = new LiftController();
        AutoController5_1 autoController51 = new AutoController5_1();
        AllCycleController allCycleController = new AllCycleController();

        double currentVoltage;
        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        currentVoltage = batteryVoltageSensor.getVoltage();
        double x1=0,y1=0,x2=0;
        double loopTime = 0;
        boolean motorColectareExtension = false;
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;

        servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.INITIALIZE;
        angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.VERTICAL;
        motorColectareController.CurrentStatus = MotorColectareController.MotorColectare.RETRACTED;
        closeClawController.CurrentStatus = CloseClawController.closeClawStatus.CLOSED;
        turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.COLLECT;
        robotController.CurrentStatus = RobotController.RobotControllerStatus.START;
        liftController.CurrentStatus = LiftController.LiftStatus.BASE;
        sigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.TRANSFER;
        autoController51.CurrentStatus = NOTHING;
        autoController51.PreviousStatus = NOTHING;
        allCycleController.CurrentStatus = AllCycleController.AllCycleControllerStatus.NOTHING;

        robotController.timerTransfer = 0.35;

        closeClawController.update(robot);
        turnClawController.update(robot);
        angle4BarController.update(robot);
        servo4BarController.update(robot);
        sigurantaLiftController.update(robot);
        motorColectareController.update(robot,0, 0.6);
        liftController.update(robot,0,sigurantaLiftController,currentVoltage);
        robotController.update(robot,sigurantaLiftController,angle4BarController,servo4BarController,motorColectareController,closeClawController,turnClawController);
        biggerController.update(robotController,closeClawController,motorColectareController);

        autoController51.Cone_Stack_Level  =5;
        autoController51.AutoLiftStatus = LiftController.LiftStatus.HIGH;
        autoController51.LimitLift = 0.85;

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        imu.initialize(parameters);
        DcMotor rightFront = null;
        DcMotor rightBack = null;
        DcMotor leftFront = null;
        DcMotor leftBack = null;
        rightFront = hardwareMap.get(DcMotor.class,"leftFront");
        leftFront = hardwareMap.get(DcMotor.class,"rightFront");
        rightBack = hardwareMap.get(DcMotor.class,"leftBack");
        leftBack = hardwareMap.get(DcMotor.class,"rightBack");


        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        double lim = 1 ; /// limita vitezei la sasiu
        boolean StrafesOn = false;
        String typeOfDrive = "RobotCentric";
        while (opModeIsActive()) {
            double hello = imu.getAngularVelocity().xRotationRate;
                previousGamepad1.copy(currentGamepad1);
                previousGamepad2.copy(currentGamepad2);

                currentGamepad1.copy(gamepad1);
                currentGamepad2.copy(gamepad2);



                if (!previousGamepad1.touchpad && currentGamepad1.touchpad)
                {
                    StrafesOn = !StrafesOn;
                }
            /// DRIVE
            if (typeOfDrive == "RobotCentric") {
                robotCentricDrive(leftFront, leftBack, rightFront, rightBack, lim,StrafesOn);
            } else {
                fieldCentricDrive(imu, leftFront, leftBack, rightFront, rightBack);
            }

            if (timeGetVoltage.seconds()>5)
            {
                timeGetVoltage.reset();
                currentVoltage = batteryVoltageSensor.getVoltage();
            }


            if ((!previousGamepad2.right_bumper && currentGamepad2.right_bumper)||(!previousGamepad1.right_bumper && currentGamepad1.right_bumper))
            {
               if (closeClawController.CurrentStatus == CloseClawController.closeClawStatus.CLOSED)
               {
                   closeClawController.CurrentStatus = CloseClawController.closeClawStatus.OPEN;
               }
               else{
                   closeClawController.CurrentStatus = CloseClawController.closeClawStatus.CLOSED;
               }
            }
            if (!previousGamepad1.square && currentGamepad1.square)
            {
                robot.left4Bar.setPosition(Servo4BarController.Fifth_Cone_Position);
                robot.right4Bar.setPosition(Servo4BarController.Fifth_Cone_Position);
                Servo4BarController.CurrentStatus = STACK_POSITION;
            }
            if (!previousGamepad1.triangle && currentGamepad1.triangle)
            {
                robot.left4Bar.setPosition(Servo4BarController.Fourth_Cone_Position);
                robot.right4Bar.setPosition(Servo4BarController.Fourth_Cone_Position);
                Servo4BarController.CurrentStatus = STACK_POSITION;
            }
            if (!previousGamepad1.circle && currentGamepad1.circle)
            {
                robot.left4Bar.setPosition(Servo4BarController.Third_Cone_Position);
                robot.right4Bar.setPosition(Servo4BarController.Third_Cone_Position);
                Servo4BarController.CurrentStatus = STACK_POSITION;
            }
            if (!previousGamepad1.cross && currentGamepad1.cross)
            {
                robot.left4Bar.setPosition(Servo4BarController.Second_Cone_Position);
                robot.right4Bar.setPosition(Servo4BarController.Second_Cone_Position);
                Servo4BarController.CurrentStatus = STACK_POSITION;
            }
            if (currentGamepad2.left_trigger>0)
            {
                if ((!previousGamepad2.dpad_up && currentGamepad2.dpad_up))
                {
                    servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.DRIVE_POSITION;
                }
                if ((!previousGamepad2.dpad_down && currentGamepad2.dpad_down))
                {
                    servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.LOW_POSITION;
                    angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus
                            .PLACE_LOW;
                }
                if ((!previousGamepad2.dpad_left && currentGamepad2.dpad_left))
                {
                    AutoController5_1.CurrentStatus = AutoController5_1.autoControllerStatus.STACK_LEVEL;
                }

            }
            else
            {
                if ((!previousGamepad2.dpad_down && currentGamepad2.dpad_down))
                {
                    robotController.CurrentStatus = RobotController.RobotControllerStatus.GO_COLLECT;
                    servo4BarController.Collect_Position = servo4BarController.Collect_Drive;
                }
                if ((!previousGamepad2.dpad_up && currentGamepad2.dpad_up))
                {
                    robotController.CurrentStatus = RobotController.RobotControllerStatus.GO_PLACE;
                }
                if ((!previousGamepad2.dpad_right && currentGamepad2.dpad_right))
                {
                    robot.left4Bar.setPosition(servo4BarController.groundJunctionPosition);
                    robot.right4Bar.setPosition(servo4BarController.groundJunctionPosition);
                    servo4BarController.CurrentStatus = STACK_POSITION;
                }
                if ((!previousGamepad2.dpad_left && currentGamepad2.dpad_left))
                {
                    robotController.CurrentStatus = RobotController.RobotControllerStatus.GO_PLACE_STACK;
                }
            }
            if (!previousGamepad2.cross && currentGamepad2.cross)
            {
                if (liftController.CurrentStatus!= LiftController.LiftStatus.HIGH)
                {
                    liftController.CurrentStatus = LiftController.LiftStatus.HIGH;
                }
                else
                {
                    liftController.CurrentStatus = LiftController.LiftStatus.BASE;
                }
            }
            if (!previousGamepad2.square && currentGamepad2.square  )
            {
                if (liftController.CurrentStatus!= LiftController.LiftStatus.BASE)
                {
                    liftController.CurrentStatus = LiftController.LiftStatus.BASE;
                }
                else
                {
                    liftController.CurrentStatus = LiftController.LiftStatus.BASE;
                }
            }
            if ( (!previousGamepad2.triangle && currentGamepad2.triangle) )
            {
                if (liftController.CurrentStatus!= LiftController.LiftStatus.LOW)
                {
                    liftController.CurrentStatus = LiftController.LiftStatus.LOW;
                }
                else
                {
                    liftController.CurrentStatus = LiftController.LiftStatus.BASE;
                }
            }
            if ( (!previousGamepad2.circle && currentGamepad2.circle) )
            {
                if (liftController.CurrentStatus!= LiftController.LiftStatus.MID)
                {
                    liftController.CurrentStatus = LiftController.LiftStatus.MID;
                }
                else
                {
                    liftController.CurrentStatus = LiftController.LiftStatus.BASE;
                }
            }
            if ((!previousGamepad2.left_bumper && currentGamepad2.left_bumper))
            {
                if (motorColectareController.CurrentStatus == MotorColectareController.MotorColectare.RETRACTED)
                {
                    motorColectareController.CurrentStatus = MotorColectareController.MotorColectare.EXTENDED;
                }
                else
                {
                    motorColectareController.CurrentStatus = MotorColectareController.MotorColectare.RETRACTED;
                }
            }
            if ((!previousGamepad1.dpad_down && currentGamepad1.dpad_down))
            {
                /*angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.COLLECT_CONES;
                servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.FALLEN_CONES;
                turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.COLLECT;*/
                if (angle4BarController.CurrentStatus == Angle4BarController.angle4BarStatus.VERTICAL)
                {
                    angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.COLLECT_CONES;
                }
                else
                {
                    angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.VERTICAL;
                }
            }

            int ColectarePosition = robot.encoderMotorColectare.getCurrentPosition();
            int LiftPosition = robot.dreaptaLift.getCurrentPosition(); /// folosesc doar encoderul de la dreaptaLift , celalalt nu exista.

            if (servo4BarController.CurrentStatus == Servo4BarController.ServoStatus.COLLECT_DRIVE)
            {
                PrecisionDenominator2=2.75;
                PrecisionDenominator = 1;
            }
            else
            if (liftController.CurrentStatus != LiftController.LiftStatus.BASE)
            {
                PrecisionDenominator2=2.75;
                PrecisionDenominator = 1.5;
            }
            else
            {
                PrecisionDenominator = 1;
                PrecisionDenominator2 = 1.5;
            }
            if (gamepad1.left_trigger > 0)
            {
                PrecisionDenominator = 2;
            }
            biggerController.update(robotController,closeClawController,motorColectareController);
            robotController.update(robot,sigurantaLiftController,angle4BarController,servo4BarController,motorColectareController,closeClawController,turnClawController);
            closeClawController.update(robot);
            turnClawController.update(robot);
            servo4BarController.update(robot);
            sigurantaLiftController.update(robot);
            motorColectareController.update(robot,ColectarePosition, 0.75);
            liftController.update(robot,LiftPosition,sigurantaLiftController,currentVoltage);
            autoController51.update(sigurantaLiftController,robot,angle4BarController,turnClawController, liftController, servo4BarController, robotController, closeClawController, motorColectareController);
            allCycleController.update(robot, sigurantaLiftController, angle4BarController,turnClawController, liftController, servo4BarController, robotController, closeClawController, motorColectareController);
            angle4BarController.update(robot);

            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.addData("XRotationRate",hello);
            telemetry.addData("AutoStatus", autoController51.CurrentStatus);
            telemetry.addData("RobotStatus",robotController.CurrentStatus);
            telemetry.addData("CurrentStatus",servo4BarController.CurrentStatus);
            telemetry.addData("salut", servo4BarController.salut);
            telemetry.addData("PreviousStatus",servo4BarController.PreviousStatus);
            telemetry.addData("WhereFromIntermediary",servo4BarController.WhereFromIntermediary);
            telemetry.addData("Timer4Bar",servo4BarController.time.seconds());
            telemetry.addData("salut",salut);
            telemetry.addData("4Barpos",robot.left4Bar.getPosition());
            telemetry.addData("CloseClawPosition",robot.closeClaw.getPosition());
            telemetry.addData("CurrentStatusCloseClaw",closeClawController.CurrentStatus);
            telemetry.addData("TurnClawPosition",robot.turnClaw.getPosition());
            telemetry.addData("Angle4BarPosition",robot.angle4Bar.getPosition());
            telemetry.addData("CurrentStatusTurnClawPosition",turnClawController.CurrentStatus);
            telemetry.addData("posColectare", ColectarePosition);
            telemetry.addData("posLift",LiftPosition);
            telemetry.addData("DreaptaPutereLift",robot.dreaptaLift.getPower());
            telemetry.addData("timpFSM", servo4BarController.time.seconds());
            telemetry.addData("SigurantaLiftStatus",sigurantaLiftController.CurrentStatus);
            telemetry.addData("sigurantaLiftPosition",robot.sigurantaLift.getPosition());
            telemetry.update();
        }
    }
}
