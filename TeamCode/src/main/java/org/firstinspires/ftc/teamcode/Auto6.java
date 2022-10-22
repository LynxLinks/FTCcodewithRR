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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
@Autonomous(name = "Auto6", group="Auto")

public class Auto6 extends LinearOpMode {
    DcMotor M0;
    FtcDashboard dashboard;
    DcMotor M1;
    DcMotor M2;
    DcMotor M3;
    DcMotor M0_2;
    Servo S0;
    DigitalChannel D0;
    DistanceSensor D1;
    DistanceSensor D2;
    DistanceSensor D4;
    public static double yoffset = 5;  //constant added to all y positions
    public static double d = 12;  //diagonal distance forward and backward
    public static double x1 = 4;
    public static double y1 = 53;
    public static double h = 800; //starting stack height
    public static double i = 50; //layer height

    int y = 2;   //y coordinate input
    int x = 0;   //x coordinate input
    double target; //slide target position
    double vy = 1;  //vector roadrunner x value
    double vx = 1;  //vector roadrunner y value
    double vo = 1;  //target roadrunner theta
    double xi = -.5;  //initial robot position against wall in coordinate system, either .5 or -.5
    int[] hdata = new int[]{200, 1100, 200, 1100, 200,
            1100, 1750, 2350, 1750, 1100,
            200, 2350, 200, 2350, 200,
            1100, 1750, 2350, 1750, 1100,
            200, 1100, 200, 1100, 200};
    int[] cords = new int[]{
            -1 , 1,
            0 , 1  ,
            -1 , 2 ,
            0 , 2 ,
            0 , 2 ,
    };
    boolean atwall = true; //used to know whether to run to or from
    boolean slidecalibrated = false;
    boolean slidecalfiller = true;
    int loop = 0; //variable for number of cycles

    //distance vars
    double p = 0;   //position
    double t = -12; //target
    double s = 0; //speed
    double f = 141; //field size
    int zone = 1;
    Trajectory t1;
    Trajectory f1;

    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        M0 = hardwareMap.get(DcMotor.class, "M0");
        M1 = hardwareMap.get(DcMotor.class, "M1");
        M2 = hardwareMap.get(DcMotor.class, "M2");
        M3 = hardwareMap.get(DcMotor.class, "M3");
        M0_2 = hardwareMap.get(DcMotor.class, "M0_2");
        S0 = hardwareMap.get(Servo.class, "S0");
        D0 = hardwareMap.get(DigitalChannel.class, "D0");
        D1 = hardwareMap.get(DistanceSensor.class, "D1");
        D2 = hardwareMap.get(DistanceSensor.class, "D3");
        D4 = hardwareMap.get(DistanceSensor.class, "D4");
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
        M0_2.setDirection(DcMotor.Direction.FORWARD);
        M0_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M0_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M0_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(D4.getDistance(DistanceUnit.INCH) - x1)
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineToSplineHeading(new Pose2d(-y1, -(D4.getDistance(DistanceUnit.INCH) - x1), Math.toRadians(90)), Math.toRadians(0))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .forward(x1)
                .build();
        S0.setPosition(0.3);

        waitForStart();
        if (isStopRequested()) return;
        //vuforia
        //zone = based on vuforia
        if (D4.getDistance(DistanceUnit.INCH) < f / 2) {
            //right
            //drive to parking zone, calibrate slide, raise slide to 200
            drive.followTrajectoryAsync(traj1);
            DriveUpdate();
            //unclamp, set slide 800, move to in front of cone stack
            S0.setPosition(0);
            target = h;
            drive.followTrajectoryAsync(traj2);
            DriveUpdate();
            //move to cone stack then center and set slide to to level
            drive.followTrajectoryAsync(traj3);
            DriveUpdate();
            Cycle();
            Park();
        }
        else{
            //left
        }
    }
    public void DriveUpdate(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        while (drive.isBusy()) {
            drive.update();
            Slide();
        }
    }
    public void Cycle(){
        while (loop < 5);
            target = h - i*loop;
            Center(); //cetner using distance sensors
            UntilSlide(); //way faster slide speed but will wait until value is hit
            S0.setPosition(0.3); //clamp
            target = h - i*loop + 200; //set slide to lift cone above stack
            UntilSlide();   //wait for slide to go up
            x = cords[2*loop];  //set new cords based on preset cords list
            y = cords[2*loop + 1];
            Drive(); //to
            S0.setPosition(0);
            if (loop < 4)Drive(); //from if its not the last cycle
            loop += 1;
    }
    public void Park(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Trajectory traj3 = drive.trajectoryBuilder(t1.end())
                .splineToConstantHeading(new Vector2d(vy, vx), Math.toRadians(vo))
                .splineToConstantHeading(new Vector2d(vy + (24* (zone-2) + .1), vx), Math.toRadians(vo))
                .build();
        DriveUpdate();

    }
    public void UntilSlide() {
        while (Math.abs(target - M0_2.getCurrentPosition()) > 10) {
            M0_2.setPower(-.7 * ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition())))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()))))));
        }
    }
    public void Slide() {
        if (!slidecalibrated){
            if (D0.getState() == true && slidecalfiller) {
                M0_2.setPower(.3);
            }
            if (D0.getState() == false) {
                M0_2.setPower(-0.3);
                slidecalfiller = false;

            }
            if (D0.getState() == true && !slidecalfiller){
                slidecalibrated = true;
                target = 200;
            }
        }


        M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250)))));

        if (D0.getState() && (target == 0)) {
            M0_2.setDirection(DcMotor.Direction.FORWARD);
            M0_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            M0_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            M0_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void Center(){
        //d2 = right
        //d4 = left

        while (gamepad2.dpad_up) {
            Slide();
            p = (D2.getDistance(DistanceUnit.INCH) + (f - D4.getDistance(DistanceUnit.INCH))) / 2;
            s = (-1 * ((1 - Math.pow(10, (((t+f/2) - p) / 100))) / (1 + Math.pow(10, (((t+f/2) - p) / 100)))));
            M0.setPower(.7*s);
            M1.setPower(s);
            M2.setPower(-s);
            M3.setPower(-.7*s);


        }

    }

    public void Drive() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
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
                target = hdata[x + 5*(y-1)-2];
                Trajectory t1 = drive.trajectoryBuilder(new Pose2d())
                        .splineToSplineHeading(new Pose2d(vy, 0, Math.toRadians(vo)),Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(vy, vx), Math.toRadians(vo))
                        .splineToConstantHeading(new Vector2d(vy + (d * Math.cos(Math.toRadians(vo))), vx + (d * Math.sin(Math.toRadians(vo)))), Math.toRadians(vo))
                        .build();
                Trajectory f1 = drive.trajectoryBuilder(t1.end())
                        .splineToConstantHeading(new Vector2d(vy, vx), Math.toRadians(vo))
                        .splineToConstantHeading(new Vector2d(vy, 0), Math.toRadians(vo))
                        .splineToSplineHeading(new Pose2d(0, 0, Math.toRadians(0)), Math.toRadians(0))
                        .build();

                drive.followTrajectoryAsync(t1);
                drive.update();
                while (drive.isBusy()) {
                    drive.update();
                    Slide();
                    while (gamepad1.right_stick_button){
                        M0.setPower(0);
                        M3.setPower(0);
                        M1.setPower(0);
                        M2.setPower(0);
                    }
                }
                atwall = false;
//
            }
            if (atwall == false) {
                S0.setPosition(0);
                target = 200;
                drive.followTrajectoryAsync(f1);
                drive.update();

                while (drive.isBusy()) {
                    drive.update();
                    Slide();

                }
                atwall = true;
            }
    }
}

