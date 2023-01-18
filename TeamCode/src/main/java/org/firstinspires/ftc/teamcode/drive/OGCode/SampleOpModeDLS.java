package org.firstinspires.ftc.teamcode.drive.OGCode;


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
    double pozTurnClaw0=0, pozTurnClaw1 = 0.78;
    boolean isDown = true, isClosed=false, isTurned = false, isExtended = false;
    double  PrecisionDenominator=1, PrecisionDenominator2=1.25;

    public void robotCentricDrive(DcMotor leftFront,DcMotor leftBack,DcMotor rightFront,DcMotor rightBack, double  lim)
    {
        double y = gamepad1.right_stick_y; // Remember, this is reversed!
        double x = 0; // Counteract imperfect strafing
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
    double Clip(double Speed,double lim)
    {
        return Math.max(Math.min(Speed,lim),-lim);
    }
    @Override
    public void runOpMode() {

        RobotMap robot=new RobotMap(hardwareMap);
        Servo4BarController servo4BarController = new Servo4BarController();
        double x1=0,y1=0,x2=0;
        double loopTime = 0;
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;

        servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.INITIALIZE;
        servo4BarController.update(robot);

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

            if (!previousGamepad1.right_bumper && currentGamepad1.right_bumper)
            {
                if (isDown) {
                    robot.left4Bar.setPosition(pozInter4Bar);
                    robot.right4Bar.setPosition(pozInter4Bar);
                    isDown=false;
                }
                else {
                    servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.COLLECT_DRIVE;
                    isDown=true;
                }
            }
            if (!previousGamepad1.b && currentGamepad1.b)
            {
               if (!isClosed) {
                   robot.closeClaw.setPosition(pozCloseClaw);
                   isClosed=true;
               }
               else{
                   robot.closeClaw.setPosition(pozOpenClaw);
                   isClosed=false;
               }
            }
            if (!previousGamepad1.y && currentGamepad1.y)
            {
                if (!isTurned){
                    robot.turnCLaw.setPosition(pozTurnClaw1);
                    isTurned=true;
                }
                else {
                    robot.turnCLaw.setPosition(pozTurnClaw0);
                    isTurned = false;

                }
            }
            if (!previousGamepad1.left_bumper && currentGamepad1.left_bumper)
            {
                if (!isExtended) {
                    servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.COLLECT_DRIVE;
                    isExtended=true;
                }
                else
                {
                    servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.PLACE_CONE;
                    isExtended=false;
                }
            }


            servo4BarController.update(robot);

            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.addData("CurrentStatus",servo4BarController.CurrentStatus);
            telemetry.addData("PreviousStatus",servo4BarController.PreviousStatus);
            telemetry.addData("WhereFromIntermediary",servo4BarController.WhereFromIntermediary);
            telemetry.addData("salut",salut);
            telemetry.addData("4Barpos",robot.left4Bar.getPosition());
            telemetry.addData("posColectare", robot.motorColectare.getCurrentPosition());
            telemetry.addData("timpFSM", servo4BarController.time.seconds());
            telemetry.update();
        }
    }
}
