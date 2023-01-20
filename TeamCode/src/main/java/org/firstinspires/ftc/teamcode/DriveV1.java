package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//name and class
@Disabled
@TeleOp(name = "DriveV1", group="Linear Opmode")


public class DriveV1 extends OpMode {

    //Define Motors
    double target;
    DcMotor M0;
    DcMotor M1;
    DcMotor M2;
    DcMotor M3;
    DcMotor M0_2;
    Servo S0;
    DigitalChannel D0;
    DigitalChannel D1;
    ColorSensor C1;
    public static double x1 = -40;
    public static double y1 = -12;
    public boolean initial = true;


    public void Untilslide(){
        while (Math.abs(target - M0_2.getCurrentPosition()) > 10){
            M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250)))));

        }
    }
    public void Math(){
        y = ycord;
        x = xcord;
        vy = -(yoffset + 24 * (y - 1));
        if (x > 0) {
            vx = .1 + 24 * Math.floor(Math.abs(x - xi));
        } else {
            vx = .1 - 24 * Math.floor(Math.abs(x - xi));
        }
        if (x > xi) {
            vo = 135;
        } else {
            vo = -135;
        }
    }

    public void Slide(){
        if (target > M0_2.getCurrentPosition()){
            M0_2.setPower(.5);
        }else{
            M0_2.setPower(-.5);
        }
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
                .lineToLinearHeading(new Pose2d(y1, -x1, Math.toRadians(0)))
                .build();
        Trajectory left1 = drive.trajectoryBuilder(left4.end())
                .lineToLinearHeading(new Pose2d(y1, 0, Math.toRadians(90)))
                .build();
        Trajectory left2 = drive.trajectoryBuilder(left1.end())
                .forward(y1)
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
                .forward(y1)
                .build();
        if (gamepad1.dpad_right) {
            if (initial) {
                S0.setPosition(.51);
                target = 30;
                Untilslide();
                target = 2300;
                M0_2.setPower(.4);
                drive.followTrajectory(right1i);
                Untilslide();
                drive.followTrajectory(right2i);
                initial = false;
            }
            else {
                S0.setPosition(0);
                target = 0;
                M0_2.setPower(-.4);
                drive.followTrajectory(right3);
                drive.followTrajectory(right4);
                Untilslide();
                S0.setPosition(.51);
                target = 2300;
                M0_2.setPower(.4);
                drive.followTrajectory(right1);
                Untilslide();
                drive.followTrajectory(right2);


            }
        }
        if (gamepad1.dpad_left) {
            if (initial) {
                S0.setPosition(.51);
                target = 2300;
                M0_2.setPower(.4);
                drive.followTrajectory(left1i);
                Untilslide();
                drive.followTrajectory(left2i);
                initial = false;
            }
            else {
                S0.setPosition(0);
                target = 0;
                drive.followTrajectory(left3);
                drive.followTrajectory(left4);
                Untilslide();
                S0.setPosition(.51);
                target = 2300;
                M0_2.setPower(.4);
                drive.followTrajectory(left1);
                Untilslide();
                drive.followTrajectory(left2);
            }
        }
    }

    public void ServoClamp() {

        //if (D0.getState() == true) S0.setPosition(.63);

        if (gamepad1.left_bumper) S0.setPosition(0);
        if (gamepad1.right_bumper) S0.setPosition(.51);
        if((D1.getState() == true) && (gamepad1.left_bumper == false)) S0.setPosition(.51);
    }
    //drive loop
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
            M0_2.setPower(-0.05);
        }
        else {
            M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250)))));
        }
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


    //init sequence
    @Override
    public void init() {
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

    }

    //loop while init
    @Override
    public void init_loop() {

    }

    //runs once after start is pressed
    @Override
    public void start(){
        target = 0;
        if  ((600 < C1.red() ) & (C1.red() < 950)){
            telemetry.addData("1",C1.red());

        }
        if  ((950 < C1.red())){
            telemetry.addData("2",C1.red());

        }
        if  (C1.red() < 600){
            telemetry.addData("3",C1.red());

        }

    }

    //looping program after start
    @Override
    public void loop() {
        MoveDriveTrain();
        ServoClamp();
        RoadRunner();
    }

}
