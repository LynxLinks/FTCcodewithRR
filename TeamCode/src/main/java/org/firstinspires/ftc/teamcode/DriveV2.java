package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Auto4.X1;
import static org.firstinspires.ftc.teamcode.Auto4.X2;
import static org.firstinspires.ftc.teamcode.Auto4.Y1;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.google.gson.annotations.Until;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.DeviceConfiguration;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//name and class
@Config
@TeleOp(name = "DriveV2", group="Linear Opmode")


public class DriveV2 extends LinearOpMode {
    double target;
    DcMotor M0;
    FtcDashboard dashboard;
    DcMotor M1;
    DcMotor M2;
    DcMotor M3;
    DcMotor M0_2;
    Servo S0;
    DigitalChannel D0;
    DigitalChannel D1;
    ColorSensor C1;
    public static double x1 = -40.5;
    public static double y1 = -9;
    public boolean initial = true;

    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        //Add Motors
        M0 = hardwareMap.get(DcMotor.class,"M0");
        M1 = hardwareMap.get(DcMotor.class,"M1");
        M2 = hardwareMap.get(DcMotor.class,"M2");
        M3 = hardwareMap.get(DcMotor.class,"M3");
        M0_2 = hardwareMap.get(DcMotor.class,"M0_2");
        S0 = hardwareMap.get(Servo.class,"S0");
        D0 = hardwareMap.get(DigitalChannel.class,"D0");
        D1 = hardwareMap.get(DigitalChannel.class,"D1");
        C1 = hardwareMap.get(ColorSensor.class, "C1");

        //Set Motors

        M0.setDirection(DcMotor.Direction.FORWARD);
        M0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        M1.setDirection(DcMotor.Direction.FORWARD);
        M1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        M2.setDirection(DcMotor.Direction.FORWARD);
        M2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        M3.setDirection(DcMotor.Direction.FORWARD);
        M3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        M0_2.setDirection(DcMotor.Direction.FORWARD);
        M0_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M0_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M0_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        C1.enableLed(true);

        waitForStart();

        while (opModeIsActive()) {
            MoveDriveTrain();
            ServoClamp();
            RoadRunner();
        }
    }
    public void Untilslide(){
        while (Math.abs(target - M0_2.getCurrentPosition()) > 10){
            M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250)))));

        }
        return;
    }
    public void RoadRunner() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory left1i = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(x1, 0, Math.toRadians(90)))
                .build();
        Trajectory left2i = drive.trajectoryBuilder(left1i.end())
                .forward(-y1)
                .build();
        Trajectory left3 = drive.trajectoryBuilder(new Pose2d())
                .back(-y1)
                .build();
        Trajectory left4 = drive.trajectoryBuilder(left3.end())
                .lineToLinearHeading(new Pose2d(y1, x1, Math.toRadians(270)))
                .build();
        Trajectory left1 = drive.trajectoryBuilder(left4.end())
                .lineToLinearHeading(new Pose2d(y1, 0, Math.toRadians(0)))
                .build();
        Trajectory left2 = drive.trajectoryBuilder(left1.end())
                .forward(-y1)
                .build();


        Trajectory right1i = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(x1, 0, Math.toRadians(270)))
                .build();
        Trajectory right2i = drive.trajectoryBuilder(right1i.end())
                .forward(-y1)
                .build();
        Trajectory right3 = drive.trajectoryBuilder(new Pose2d())
                .back(-y1)
                .build();
        Trajectory right4 = drive.trajectoryBuilder(right3.end())
                .lineToLinearHeading(new Pose2d(y1, -x1, Math.toRadians(90)))
                .build();
        Trajectory right1 = drive.trajectoryBuilder(right4.end())
                .lineToLinearHeading(new Pose2d(y1, 0, Math.toRadians(0)))
                .build();
        Trajectory right2 = drive.trajectoryBuilder(right1.end())
                .forward(-y1)
                .build();
        if (gamepad1.dpad_right) {
            if (initial) {
                S0.setPosition(.51);
                target = 30;
                Untilslide();
                target = 2350;
                M0_2.setPower(.25);
                drive.followTrajectory(right1i);
                Untilslide();
                M0_2.setPower(0);
                drive.followTrajectory(right2i);
                initial = false;
            } else {
                S0.setPosition(0);
                target = 0;
                M0_2.setPower(-.3);
                drive.followTrajectory(right3);
                drive.followTrajectory(right4);
                Untilslide();
                initial = true;


            }
        }
        if (gamepad1.dpad_left) {
            if (initial) {
                S0.setPosition(.51);
                target = 2350;
                M0_2.setPower(.25);
                drive.followTrajectory(left1i);
                Untilslide();
                M0_2.setPower(0);
                drive.followTrajectory(left2i);
                initial = false;
            }
            if (initial == false){
                S0.setPosition(0);
                target = 0;
                M0_2.setPower(-.3);
                drive.followTrajectory(left3);
                drive.followTrajectory(left4);
                Untilslide();
                initial = true;
            }
        }
    }
    public void ServoClamp() {

        //if (D0.getState() == true) S0.setPosition(.63);

        if (gamepad1.left_bumper) S0.setPosition(0);
        if (gamepad1.right_bumper) S0.setPosition(.51);
        if((D1.getState() == true) && (gamepad1.left_bumper == false)) S0.setPosition(.51);
    }
    public void MoveDriveTrain(){
        //drive variables
        double yAxis;
        double xAxis;
        double Rotate;

        //input to change variables
        yAxis = gamepad1.left_stick_y + gamepad1.right_stick_y/3;
        xAxis = gamepad1.left_stick_x + gamepad1.right_stick_x/3;
        Rotate = -gamepad1.left_trigger+gamepad1.right_trigger;

//dick
        //apply variables to motor
        M0.setPower((Rotate + (-yAxis + xAxis)));
        M3.setPower((Rotate + (-yAxis - xAxis)));
        M1.setPower(-(Rotate + (yAxis + xAxis)));
        M2.setPower(-(Rotate + (yAxis - xAxis)));

        //dowm
        if (gamepad1.a) {
            target = 0;
        }
        //up
        if(gamepad1.b) {
            target = 2300;
        }
        if(gamepad1.y) {
            target = 1600;
        }
        if(gamepad1.x) {
            target = 950;
        }
        if(D0.getState() && (target == 0)){
            M0_2.setDirection(DcMotor.Direction.FORWARD);
            M0_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            M0_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            M0_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if(target==0 && D0.getState() == false && M0_2.getCurrentPosition() <= 0){
            M0_2.setPower(-0.05);}


        M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250)))));

        telemetry.addData("current ",M0_2.getCurrentPosition());
        telemetry.addData("delta", target - M0_2.getCurrentPosition());
        telemetry.addData("target",target);
        telemetry.addData("equation",-1 * ((1 - Math.pow( 10,((target - M0_2.getCurrentPosition())/500)))/(1 + Math.pow( 10,((target - M0_2.getCurrentPosition())/500)))));
        telemetry.addData("clamp ",D1.getState());
        telemetry.addData("slide ",D0.getState());
        telemetry.addData("servo shit",S0.getPosition() );
        telemetry.addData("red", C1.red());
        telemetry.addData("green", C1.green());
        telemetry.addData("blue", C1.blue());
        telemetry.update();
        //dick


    }
}
