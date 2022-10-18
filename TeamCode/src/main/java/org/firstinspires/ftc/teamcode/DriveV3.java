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
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.DeviceConfiguration;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//name and class
@Config
@TeleOp(name = "DriveV3", group="Linear Opmode")

public class DriveV3 extends LinearOpMode {
    double target;
    DcMotor M0;
    FtcDashboard dashboard;
    DcMotor M1;
    DcMotor M2;
    DcMotor M3;
    DcMotor M0_2;
    Servo S0;
    DigitalChannel D0;
    DistanceSensor D1;
    double yCord = 2;
    public static double x1 = -28;
    public static double y1 = 12;
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
        D1 = hardwareMap.get(DistanceSensor.class,"D1");

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

        waitForStart();

        while(D0.getState() == true){
            M0_2.setPower(.3);
        }
        while(D0.getState() == false){
            M0_2.setPower(-0.05);
        }
        M0_2.setDirection(DcMotor.Direction.FORWARD);
        M0_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M0_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M0_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        S0.setPosition(0.0);

        target = 200;

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

        Trajectory right1i = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(-(5+24*(yCord-1)), 0, Math.toRadians(-135)))
                .build();
        Trajectory right2i = drive.trajectoryBuilder(right1i.end())
                .forward(y1)
                .build();
        Trajectory right3 = drive.trajectoryBuilder(new Pose2d())
                .back(y1)
                .build();
        Trajectory right4 = drive.trajectoryBuilder(right1i.end())
                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)))
                .build();

        if (gamepad1.dpad_right) {
            if (initial) {
                S0.setPosition(.33);
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
    }
    public void ServoClamp() {

        //if (D0.getState() == true) S0.setPosition(.63);

        if (gamepad1.left_bumper) S0.setPosition(0.0);
        if (gamepad1.right_bumper){
            target = 5;
            Untilslide();
            S0.setPosition(0.3);
            while(D0.getState() == false){
                M0_2.setPower(-0.05);
            }
            M0_2.setDirection(DcMotor.Direction.FORWARD);
            M0_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            M0_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            M0_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if((target == 200) && (D1.getDistance(DistanceUnit.METER) <= .033)){
            target = 5;
            Untilslide();
            S0.setPosition(0.3);
            while(D0.getState() == false){
                M0_2.setPower(-0.05);
            }
            M0_2.setDirection(DcMotor.Direction.FORWARD);
            M0_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            M0_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            M0_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void BuildTraject(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Trajectory left1i = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(x1, 0, Math.toRadians(90)))
                .build();
        Trajectory left2i = drive.trajectoryBuilder(left1i.end())
                .forward(-y1)
                .build();
        Trajectory left3 = drive.trajectoryBuilder(left2i.end())
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
                .lineToLinearHeading(new Pose2d(6+24*yCord, 0, Math.toRadians(-135)))
                .build();
        Trajectory right2i = drive.trajectoryBuilder(right1i.end())
                .forward(y1)
                .build();
        Trajectory right3 = drive.trajectoryBuilder(new Pose2d())
                .back(y1)
                .build();
        Trajectory right4 = drive.trajectoryBuilder(right3.end())
                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)))
                .build();

    }
    public void MoveDriveTrain(){
        //drive variables
        double yAxis;
        double xAxis;
        double Rotate;

        //input to change variables
        yAxis = gamepad1.left_stick_y*.8 + gamepad1.right_stick_y/3;
        xAxis = gamepad1.left_stick_x*.8 + gamepad1.right_stick_x/3;
        Rotate = -gamepad1.left_trigger/2 + gamepad1.right_trigger/2;

//dick
        //apply variables to motor
        M0.setPower((Rotate + (-yAxis + xAxis)));
        M3.setPower((Rotate + (-yAxis - xAxis)));
        M1.setPower(-(Rotate + (yAxis + xAxis)));
        M2.setPower(-(Rotate + (yAxis - xAxis)));

        //dowm
        if (gamepad1.a) {
            target = 200;
            S0.setPosition(0.0);
        }
        //up
        if(gamepad1.b) {
            target = 2350;
        }
        if(gamepad1.y) {
            target = 1750;
        }
        if(gamepad1.x) {
            target = 1100;
        }
        if(D0.getState() && (target == 0)){
            M0_2.setDirection(DcMotor.Direction.FORWARD);
            M0_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            M0_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            M0_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250)))));

        telemetry.addData("current ",M0_2.getCurrentPosition());
        telemetry.addData("delta", target - M0_2.getCurrentPosition());
        telemetry.addData("target",target);
        telemetry.addData("equation",-1 * ((1 - Math.pow( 10,((target - M0_2.getCurrentPosition())/500)))/(1 + Math.pow( 10,((target - M0_2.getCurrentPosition())/500)))));
        telemetry.addData("slide ",D0.getState());
        telemetry.addData("servo shit",S0.getPosition() );
        telemetry.addData("distance",D1.getDistance(DistanceUnit.METER) );
        //telemetry.addData("green", C1.green());
        //  telemetry.addData("blue", C1.blue());
        telemetry.update();
        //dick

    }
}
