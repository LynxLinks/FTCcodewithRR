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

import java.lang.reflect.Array;

//name and class
@Config
@TeleOp(name = "DriveV4", group="Linear Opmode")

public class DriveV4 extends LinearOpMode {
    DcMotor M0;
    FtcDashboard dashboard;
    DcMotor M1;
    DcMotor M2;
    DcMotor M3;
    DcMotor M0_2;
    Servo S0;
    DigitalChannel D0;
    DistanceSensor D1;
    public static double yoffset = 5;  //constant added to all y positions
    public static double d = 12;  //diagonal distance forward and backward
    int y = 1;   //y coordinate input
    int x = 0;   //x coordinate input
    double target; //slide target position
    double vy = 1;  //vector roadrunner x value
    double vx = 1;  //vector roadrunner y value
    double vo = 1;  //target roadrunner theta
    double xi = 0;  //initial robot position against wall in coordinate system, either .5 or -.5
    int[] hdata = new int[]{200, 1100, 200, 1100, 200,
            1100, 1750, 2350, 1750, 1100,
            200, 2350, 200, 2350, 200,
            1100, 1750, 2350, 1750, 1100,
            200, 1100, 200, 1100, 200};
    boolean atwall = true; //used to know whether to run to or from
    boolean dup = false;
    boolean ddown = false;
    boolean dright = false;
    boolean dleft = false;
    Trajectory t12;
    Trajectory f12;

    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        //Add Motors
        M0 = hardwareMap.get(DcMotor.class, "M0");
        M1 = hardwareMap.get(DcMotor.class, "M1");
        M2 = hardwareMap.get(DcMotor.class, "M2");
        M3 = hardwareMap.get(DcMotor.class, "M3");
        M0_2 = hardwareMap.get(DcMotor.class, "M0_2");
        S0 = hardwareMap.get(Servo.class, "S0");
        D0 = hardwareMap.get(DigitalChannel.class, "D0");
        D1 = hardwareMap.get(DistanceSensor.class, "D1");

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


        while (D0.getState() == true) {
            M0_2.setPower(.3);
        }
        while (D0.getState() == false) {
            M0_2.setPower(-0.05);
        }
        M0_2.setPower(0);
        M0_2.setDirection(DcMotor.Direction.FORWARD);
        M0_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M0_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M0_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        S0.setPosition(0.0);
        waitForStart();
        target = 200;
        while (opModeIsActive()) {
            Drive();
            ServoClamp();
            Slide();
            Coordinates();
        }
    }

    public void ServoClamp() {
        //automatic clamping if within distance
        if (gamepad1.left_bumper) {
            S0.setPosition(0.0);
        }
        if ((target == 200) && (D1.getDistance(DistanceUnit.METER) <= .033) || gamepad1.right_bumper) {
            target = 5;
            while (Math.abs(target - M0_2.getCurrentPosition()) > 10) {
                M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250)))));
            }
            M0_2.setPower(0);
            S0.setPosition(0.3);
        }
    }

    public void Slide() {
        M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250)))));
        if (gamepad1.a) {
            target = 200;
            S0.setPosition(0.0);
        }
        if (gamepad1.b) target = 2350;
        if (gamepad1.y) target = 1750;
        if (gamepad1.x) target = 1100;
        if (D0.getState() && (target == 0)) {
            M0_2.setDirection(DcMotor.Direction.FORWARD);
            M0_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            M0_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            M0_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void Drive() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        double yAxis;
        double xAxis;
        double Rotate;
        yAxis = gamepad1.left_stick_y * .8 + gamepad1.right_stick_y / 3;
        xAxis = gamepad1.left_stick_x * .8 + gamepad1.right_stick_x / 3;
        Rotate = -gamepad1.left_trigger / 2 + gamepad1.right_trigger / 2;
        M0.setPower((Rotate + (-yAxis + xAxis)));
        M3.setPower((Rotate + (-yAxis - xAxis)));
        M1.setPower(-(Rotate + (yAxis + xAxis)));
        M2.setPower(-(Rotate + (yAxis - xAxis)));

        if (gamepad1.right_stick_button) {
            if (atwall) {
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

                Trajectory t12 = drive.trajectoryBuilder(new Pose2d())
                        .splineToSplineHeading(new Pose2d(vy, 0, Math.toRadians(vo)), Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(vy, vx, Math.toRadians(vo)), Math.toRadians(vo))
                        .splineToSplineHeading(new Pose2d(vy - (d * Math.cos(Math.toRadians(vo))), vx - (d * Math.sin(Math.toRadians(vo))), Math.toRadians(vo)), Math.toRadians(vo))
                        .build();
                Trajectory f12 = drive.trajectoryBuilder(t12.end())
                        .splineToSplineHeading(new Pose2d(vy, vx, Math.toRadians(vo)), Math.toRadians(vo))
                        .splineToSplineHeading(new Pose2d(vy, 0, Math.toRadians(vo)), Math.toRadians(vo))
                        .splineToSplineHeading(new Pose2d(0, 0, Math.toRadians(0)), Math.toRadians(0))
                        .build();
                drive.followTrajectoryAsync(t12);
                drive.update();
                //while(!gamepad1.dpad_up && drive.isBusy()) {
                //drive.update();
                //Slide();
            //}
                while (Math.abs(gamepad1.left_stick_x) < .5
                        && Math.abs(gamepad1.left_stick_y) < .5
                        && Math.abs(gamepad1.right_stick_x) < .5
                        && Math.abs(gamepad1.right_stick_y) < .5) {
                    drive.update();
                    Slide();
                    while (gamepad1.right_stick_button) ;
                }
                atwall = false;
//
            }
            if (atwall == false) {
                S0.setPosition(0);
                target = 200;
                drive.followTrajectoryAsync(f12);
                drive.update();
                //while(!gamepad1.dpad_up && drive.isBusy()) {
                //drive.update();
                //Slide();
                //}
                while (Math.abs(gamepad1.left_stick_x) < .5
                        && Math.abs(gamepad1.left_stick_y) < .5
                        && Math.abs(gamepad1.right_stick_x) < .5
                        && Math.abs(gamepad1.right_stick_y) < .5) {
                    drive.update();
                    Slide();
                    while (gamepad1.right_stick_button) ;
                }
                atwall = true;
            }

        }

    }
        public void Coordinates () {
            if (gamepad2.dpad_up) dup = true;
            if (gamepad2.dpad_down) ddown = true;
            if (gamepad2.dpad_left) dleft = true;
            if (gamepad2.dpad_right) dright = true;
            if (gamepad2.right_bumper) {
                xi = .5;
            }
            if (gamepad2.left_bumper) {
                xi = -.5;
            }
            if ((!gamepad2.dpad_up) && dup) {
                dup = false;
                y += 1;
            }
            if ((!gamepad2.dpad_down) && ddown) {
                ddown = false;
                y -= 1;
            }
            if ((!gamepad2.dpad_right) && dright) {
                dright = false;
                x += 1;
            }
            if ((!gamepad2.dpad_left) && dleft) {
                dleft = false;
                x -= 1;
            }
            telemetry.addData("xi", xi);
            telemetry.addData("x", x);
            telemetry.addData("y", y);

            telemetry.update();


        }
    }

