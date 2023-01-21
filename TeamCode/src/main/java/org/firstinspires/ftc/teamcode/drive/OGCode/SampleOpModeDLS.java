package org.firstinspires.ftc.teamcode.drive.OGCode;


import static org.firstinspires.ftc.teamcode.drive.OGCode.Servo4BarController.ServoStatus.PLACE_CONE;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.exception.RobotCoreException;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

@TeleOp(name="SampleOpModeDLS", group="Linear Opmode")

public class SampleOpModeDLS extends  LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    public static int salut=0;
    double pozInit4Bar = 0, pozInter4Bar= 0.4, pozPlace4Bar = 0.7;
    double pozCloseClaw=0.8, pozOpenClaw=0.2;
    double kp =0, ki=0, kd=0;
    boolean isDown = true, isClosed=false, isTurned = false, isExtended = false;
    double  PrecisionDenominator=1, PrecisionDenominator2=1.25;

    public void robotCentricDrive(DcMotor leftFront,DcMotor leftBack,DcMotor rightFront,DcMotor rightBack, double  lim)
    {
        double y = gamepad1.right_stick_y; // Remember, this is reversed!
        double x = gamepad1.right_stick_x*1.1; // Counteract imperfect strafing
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
    public void fieldCentricDrive(BNO055IMU imu,DcMotor leftFront, DcMotor leftBack, DcMotor rightFront, DcMotor rightBack)
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
        Servo4BarController servo4BarController = new Servo4BarController();
        MotorColectareController motorColectareController = new MotorColectareController();
        CloseClawController closeClawController = new CloseClawController();
        TurnClawController turnClawController = new TurnClawController();
        RobotController robotController = new RobotController();
        BiggerController biggerController = new BiggerController();
        LiftController liftController = new LiftController();
        ServoLiftController servoLiftController = new ServoLiftController();

        double x1=0,y1=0,x2=0;
        double loopTime = 0;
        boolean motorColectareExtension = false;
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;

        servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.INITIALIZE;
        motorColectareController.CurrentStatus = MotorColectareController.MotorColectare.RETRACTED;
        closeClawController.CurrentStatus = CloseClawController.closeClawStatus.CLOSED;
        turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.COLLECT;
        robotController.CurrentStatus = RobotController.RobotControllerStatus.START;
        liftController.CurrentStatus = LiftController.LiftStatus.BASE;
        servoLiftController.CurrentStatus = ServoLiftController.ServoLiftStatus.TRANSFER;

        closeClawController.update(robot);
        turnClawController.update(robot);
        servo4BarController.update(robot);
        servoLiftController.update(robot);
        motorColectareController.update(robot,0);
        liftController.update(robot,0);
        robotController.update(servoLiftController,servo4BarController,motorColectareController,closeClawController,turnClawController);
        biggerController.update(robotController,closeClawController,motorColectareController);

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

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        double lim = 1 ; /// limita vitezei la sasiu
        String typeOfDrive = "RobotCentric";
        while (opModeIsActive()) {

                previousGamepad1.copy(currentGamepad1);
                previousGamepad2.copy(currentGamepad2);

                currentGamepad1.copy(gamepad1);
                currentGamepad2.copy(gamepad2);




            /// DRIVE
            if (typeOfDrive == "RobotCentric") {
                robotCentricDrive(leftFront, leftBack, rightFront, rightBack, lim);
            } else {
                fieldCentricDrive(imu, leftFront, leftBack, rightFront, rightBack);
            }
            //robot.stangaLift.setPower(gamepad2.right_stick_y);
            //robot.dreaptaLift.setPower(gamepad2.right_stick_y);
            if ((!previousGamepad2.square && currentGamepad2.square)||(!previousGamepad1.square && currentGamepad1.square))
            {
                if (servoLiftController.CurrentStatus== ServoLiftController.ServoLiftStatus.TRANSFER)
                {
                    servoLiftController.CurrentStatus= ServoLiftController.ServoLiftStatus.JUNCTION;
                }
                else
                {
                    servoLiftController.CurrentStatus= ServoLiftController.ServoLiftStatus.TRANSFER;
                }
            }
            if ((!previousGamepad2.triangle && currentGamepad2.triangle)||(!previousGamepad1.triangle && currentGamepad1.triangle))
            {
               if (closeClawController.CurrentStatus == CloseClawController.closeClawStatus.CLOSED)
               {
                   closeClawController.CurrentStatus = CloseClawController.closeClawStatus.OPEN;
               }
               else{
                   closeClawController.CurrentStatus = CloseClawController.closeClawStatus.CLOSED;
               }
            }
            if (!previousGamepad1.cross && currentGamepad1.cross  || (!previousGamepad2.cross && currentGamepad2.cross) )
            {
                if (liftController.CurrentStatus== LiftController.LiftStatus.BASE)
                {
                    liftController.CurrentStatus = LiftController.LiftStatus.HIGH;
                }
                else
                {
                    liftController.CurrentStatus = LiftController.LiftStatus.BASE;
                }
            }
            if (!previousGamepad1.circle && currentGamepad1.circle || (!previousGamepad2.circle && currentGamepad2.circle))
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
            if ((!previousGamepad2.left_bumper && currentGamepad2.left_bumper) || (!previousGamepad1.left_bumper && currentGamepad1.left_bumper))
            {
                if (servo4BarController.CurrentStatus == PLACE_CONE) {
                    servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.COLLECT_DRIVE;
                }
                else
                {
                    servo4BarController.CurrentStatus = PLACE_CONE;
                }
            }
            if ((!previousGamepad2.dpad_down && currentGamepad2.dpad_down) || !previousGamepad1.dpad_down && currentGamepad1.dpad_down)
            {
                robotController.CurrentStatus = RobotController.RobotControllerStatus.GO_COLLECT;
            }
            if ((!previousGamepad2.dpad_up && currentGamepad2.dpad_up) || (!previousGamepad1.dpad_up && currentGamepad1.dpad_up))
            {
                robotController.CurrentStatus = RobotController.RobotControllerStatus.GO_PLACE;
            }
            if ((!previousGamepad2.dpad_left && currentGamepad2.dpad_left) || (!previousGamepad1.dpad_left && currentGamepad1.dpad_left))
            {
                biggerController.CurrentStatus = BiggerController.biggerControllerStatus.COLLECT_RAPID_FIRE;
            }
            int ColectarePosition = robot.motorColectare.getCurrentPosition();
            int LiftPosition = robot.dreaptaLift.getCurrentPosition(); /// folosesc doar encoderul de la dreaptaLift , celalalt nu exista.

            biggerController.update(robotController,closeClawController,motorColectareController);
            robotController.update(servoLiftController,servo4BarController,motorColectareController,closeClawController,turnClawController);
            closeClawController.update(robot);
            turnClawController.update(robot);
            servo4BarController.update(robot);
            servoLiftController.update(robot);
            motorColectareController.update(robot,ColectarePosition);
            liftController.update(robot,LiftPosition);

            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.addData("RobotStatus",robotController.CurrentStatus);
            telemetry.addData("CurrentStatus",servo4BarController.CurrentStatus);
            telemetry.addData("PreviousStatus",servo4BarController.PreviousStatus);
            telemetry.addData("WhereFromIntermediary",servo4BarController.WhereFromIntermediary);
            telemetry.addData("salut",salut);
            telemetry.addData("4Barpos",robot.left4Bar.getPosition());
            telemetry.addData("CloseClawPosition",robot.closeClaw.getPosition());
            telemetry.addData("CurrentStatusCloseClaw",closeClawController.CurrentStatus);
            telemetry.addData("TurnClawPosition",robot.turnClaw.getPosition());
            telemetry.addData("CurrentStatusTurnClawPosition",turnClawController.CurrentStatus);
            telemetry.addData("posColectare", ColectarePosition);
            telemetry.addData("posLift",LiftPosition);
            telemetry.addData("DreaptaPutereLift",robot.dreaptaLift.getPower());
            telemetry.addData("timpFSM", servo4BarController.time.seconds());
            telemetry.addData("servoLiftPosition",robot.servoLift.getPosition());
            telemetry.update();
        }
    }
}
