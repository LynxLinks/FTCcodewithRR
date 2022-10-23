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
@Autonomous(name = "Red", group="Auto")

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
    public static boolean red = true;
    public static int cycles = 5;

    double o1 = 0;
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
    boolean audienceside = true;
    int loop = 0; //variable for number of cycles
    //distance vars
    double p = 0;   //position
    double t = -12; //target
    double s = 0; //speed
    double f = 141; //field size
    int zone = 1;
    Trajectory t1;
    Trajectory t2;
    Trajectory t3;
    Trajectory f1;
    Trajectory f2;
    Trajectory f3;
    Trajectory i1;
    Trajectory i2;
    Trajectory i3;
    Trajectory i4;
    Trajectory p1;
    Trajectory p2;
    Trajectory p3;

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


        S0.setPosition(0.3);
        if (red){
            if (D2.getDistance(DistanceUnit.INCH) < f / 2) {                                            //red left
                x1 = -(D2.getDistance(DistanceUnit.INCH)-x1);
                y1 = - y1;
                xi = .5;
                o1 = -90;
                audienceside = true;
            }
            else {                                                                              //red right
                y1 = - y1;
                o1 = -135;
                x1 = (D4.getDistance(DistanceUnit.INCH));
                xi = -.5;
                audienceside = false;
            }
        }
        else{
            if (D2.getDistance(DistanceUnit.INCH) < f / 2){                                             //blue left
                y1 = - y1;
                o1 = 135;
                x1 = (D2.getDistance(DistanceUnit.INCH));
                xi = .5;
                audienceside = false;
            }
            else {                                                                              //blue right
                x1 = (D4.getDistance(DistanceUnit.INCH)-x1);
                y1 = - y1;
                xi = -.5;
                o1 = 90;
                audienceside = true;
            }


        }


        waitForStart();
        if (isStopRequested()) return;
        //vuforia
        //zone = based on vuforia

        Setup();
        Cycle();
        Park();
    }

    public void Setup(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        if (audienceside ){


            Trajectory i1 = drive.trajectoryBuilder(new Pose2d())
                    .strafeLeft(x1)
                    .addDisplacementMarker(() -> {
                        drive.followTrajectoryAsync(i2);
                        S0.setPosition(0);
                        target = h;
                    })
                    .build();
            Trajectory i2 = drive.trajectoryBuilder(i1.end())
                    .splineToSplineHeading(new Pose2d(y1, x1, Math.toRadians(o1)), Math.toRadians(o1))
                    .addDisplacementMarker(() -> drive.followTrajectoryAsync(i3))
                    .build();
            Trajectory i3 = drive.trajectoryBuilder(i2.end())
                    .forward(x1)
                    .build();

        }
        else{
            Trajectory i1 = drive.trajectoryBuilder(new Pose2d())
                    .splineToSplineHeading(new Pose2d(y1, 0, Math.toRadians(o1)), Math.toRadians(o1))
                    .addDisplacementMarker(() -> drive.followTrajectoryAsync(i2))
                    .build();
            Trajectory i2 = drive.trajectoryBuilder(i1.end())
                    .forward(d)
                    .addDisplacementMarker(() -> drive.followTrajectoryAsync(i3))
                    .build();
            Trajectory i3 = drive.trajectoryBuilder(i1.end())
                    .back(d)
                    .addDisplacementMarker(() -> drive.followTrajectoryAsync(i4))
                    .build();
            Trajectory i4 = drive.trajectoryBuilder(i3.end())
                    .lineToLinearHeading(new Pose2d(y1, x1, 2*vo))
                    .build();

        }
        drive.followTrajectoryAsync(i1);
        drive.update();
        DriveUpdate();

    }
    public void Cycle(){
        while (loop < cycles);{
            target = h - (i*loop);
            Center(); //cetner using distance sensors
            UntilSlide(); //way faster slide speed but will wait until value is hit
            S0.setPosition(0.3); //clamp
            target = h - i*loop + 200; //set slide to lift cone above stack
            UntilSlide();   //wait for slide to go up
            x = cords[2*loop];  //set new cords based on preset cords list
            y = cords[2*loop + 1];
            Drive(); //to
            S0.setPosition(0);
            if (loop < (cycles - 1))Drive(); //from if its not the last cycle
            loop += 1;
        }
    }
    public void Park(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Trajectory traj3 = drive.trajectoryBuilder(t1.end())
                .splineToConstantHeading(new Vector2d(vy, vx), Math.toRadians(vo))
                .splineToConstantHeading(new Vector2d(vy + ((xi/Math.abs(xi))*(24* (zone-2) + .1)), vx), Math.toRadians(vo))  //if xi = .5 then add (zone-2)*24 to y   // if xi = -.5 then subtract (zone-2)*24 from y
                .build();
        drive.followTrajectoryAsync(traj3);
        drive.update();
        DriveUpdate();

    }

    public void DriveUpdate(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        while (drive.isBusy()) {
            drive.update();
            Slide();
        }
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
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        p = (D2.getDistance(DistanceUnit.INCH) - (f - D4.getDistance(DistanceUnit.INCH)));
        Trajectory c1 = drive.trajectoryBuilder(new Pose2d(0,0,0))
                .lineToLinearHeading(new Pose2d(0, (24*xi)-p, Math.toRadians(0)))
                .build();
        drive.followTrajectoryAsync(c1);
        drive.update();
        DriveUpdate();

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
                Trajectory t1 = drive.trajectoryBuilder(new Pose2d(0,0,0))
                        .lineToLinearHeading(new Pose2d(vy, 0, Math.toRadians(0)))
                        .addDisplacementMarker(() -> drive.followTrajectoryAsync(t2))
                        .build();
                //move to x position
                Trajectory t2 = drive.trajectoryBuilder(t1.end())
                        .lineToLinearHeading(new Pose2d(vy, vx, Math.toRadians(vo)))
                        .addDisplacementMarker(() -> drive.followTrajectoryAsync(t3))
                        .build();
                //move diagonal forwards to target junction
                Trajectory t3 = drive.trajectoryBuilder(t2.end())
                        .forward(d)
                        .build();
                //move diagonal backwards to center of tile
                Trajectory f1 = drive.trajectoryBuilder(t3.end())
                        .back(d)
                        .addDisplacementMarker(() -> drive.followTrajectoryAsync(f2))
                        .build();
                //move to 0 x and 0 theta
                Trajectory f2 = drive.trajectoryBuilder(f1.end())
                        .lineToLinearHeading(new Pose2d(vy, 0, 0))
                        .addDisplacementMarker(() -> drive.followTrajectoryAsync(t3))
                        .build();
                //move to 0 y
                Trajectory f3 = drive.trajectoryBuilder(f2.end())
                        .lineToLinearHeading(new Pose2d(0, 0, 0))
                        .build();

                drive.followTrajectoryAsync(t1);
                drive.update();
                DriveUpdate();
                atwall = false;
//
            }
            if (atwall == false) {
                S0.setPosition(0);
                target = 200;
                drive.followTrajectoryAsync(f1);
                drive.update();

                DriveUpdate();
                atwall = true;
            }
    }
}

