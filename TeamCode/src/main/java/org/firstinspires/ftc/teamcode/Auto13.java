package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.google.gson.annotations.Until;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;


@Config
@Autonomous(name="Auto13", group="Linear Opmode")

public class Auto13 extends LinearOpMode {

    DcMotor M0;
    FtcDashboard dashboard;
    DcMotor M1;
    DcMotor M2;
    DcMotor M3;
    DcMotor M0_2;
    Servo S0;
    Servo S1;
    Servo S2;
    DigitalChannel D0;
    DistanceSensor D1;
    DistanceSensor D2;
    DistanceSensor D3;
    DistanceSensor D4;
    DigitalChannel D5;

    //teleop vars
    TrajectorySequence traj;
    Pose2d currentpose;
    Pose2d prevpose;
    public static double d1 = 11.5;
    public static double d2 = 3;
    public static double Sdrop = 350;
    public static boolean sidered = true;
    public static double dwall = 15;
    public static double dwall2 = -2;
    public static double ywall = 52.6;
    public static double dslam = 4;
    public static double offset = 12;
    public static double slideoffset = 100;
    public static double slidespeed = .6;
    public static double reverseoffset = 10.2      ;
    public static double bump = 150;
    public static double calibratespeed = 1;
    public static double centerpos = 50.2;
    public static double defaultcenter = 50;
    boolean useiteration = false;
boolean translate;
    public static double vopark;
    public static Pose2d autopose = new Pose2d();

    int[] xcord = new int[]{ -1,-1,0};
    int[] ycord = new int[]{2,3,2};

    int[] hdata = {100, 1150, 100, 1150, 100,
            1150, 1750, 2250, 1750, 1150,
            100, 2250, 100, 2250, 100,
            1150, 1750, 2250, 1750, 1150,
            100, 1150, 100, 1150, 100
            ,200,200,200,200,200,200,200,200,200,200,200};


    TrajectorySequence init1;
    TrajectorySequence parktraj;
    TrajectorySequence tslam;
    String zone = "3";

    int preset = 1;
    int xm = 1;
    double target;
    boolean beacon;
    double park;
    int xcordset;
    int ycordset;
    int wcordset;
    double vy;
    double vx;
    double vo;
    int y;
    int x;
    int w = 1;
    double starget;
    double d;
    double ix;
    double iy;
    double io;
    double x1;
    double x2;
    double x3;
    double y1;
    double y2;
    double y3;
    double o1;
    double o2;
    double o3;
    boolean atwall = true;
    boolean beenoff = false;
    boolean slidecalibrated = false;
    boolean usedistance = true;


    private static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/tflitemodels/model2.tflite";
    private static final String[] LABELS = {"1", "2", "3"};
    private static final String VUFORIA_KEY = "AW9YHO7/////AAABmac9s9LxrUDQuPRw+qlK4+1ZyAszPO7ouIyCLjm98NVZSbtunzGw0u8sSmhuTWNKjpGUxGCkMqV1mVUxgl9h4/J0GxK6120V5SfAcPH2XO17MGzFAm421Lcixendmv2WpyNU3HqERp0Og+sWFVwQTMM5f9rPnzsGiOVSOJ3xgZ9vltV2yHIrYxq1X95szieyo3xGab+kyy4mP5gWgu4VYwqffKb+nhXQ28jTzYhkqTbmE1saub+9juGnkNbqolX3A82q6/jrpIq1a/Nx5Egebwv1ItuABv0lq0gQJ4MiAGOf6czB9FnreVCYxSA4bUvCEZYxUG9RPgbczmU6eW85/wDskT3+1vMYR+BoqqwSa0mr";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public void runOpMode() {

        dashboard = FtcDashboard.getInstance();
        M0 = hardwareMap.get(DcMotor.class, "M0");
        M1 = hardwareMap.get(DcMotor.class, "M1");
        M2 = hardwareMap.get(DcMotor.class, "M2");
        M3 = hardwareMap.get(DcMotor.class, "M3");
        M0_2 = hardwareMap.get(DcMotor.class, "M0_2");
        S0 = hardwareMap.get(Servo.class, "S0");
        S1 = hardwareMap.get(Servo.class, "S1");
        S2 = hardwareMap.get(Servo.class, "S2");
        D0 = hardwareMap.get(DigitalChannel.class, "D0");
        D1 = hardwareMap.get(DistanceSensor.class, "D1");
        D2 = hardwareMap.get(DistanceSensor.class, "D2");
        D3 = hardwareMap.get(DistanceSensor.class, "D3");
        D4 = hardwareMap.get(DistanceSensor.class, "D4");
        D5 = hardwareMap.get(DigitalChannel.class, "D5");
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
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        initVuforia();
        initTfod();
        if (D2.getDistance(DistanceUnit.INCH)<D2.getDistance(DistanceUnit.INCH)){
            sidered = true;
            xm = 1;
            wcordset = 1;

        }
        else{
            sidered = false;
            xm = -1;
            wcordset = 4;

        }

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.0, 16.0 / 12.0);
        }

        FtcDashboard.getInstance().startCameraStream(tfod, 0);

        while (!isStarted()) {
            IdentifyVuforia();
        }



        S0.setPosition(0.05);
        S1.setPosition(0);
        S2.setPosition(0.70);
        Init();
        Cycle();
        Park();

    }

    public void Init() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        target = 900;
        if (sidered) {


            x1 = -(D2.getDistance(DistanceUnit.INCH) - dwall);
            y1 = 2;
            o1 = Math.toRadians(-90);

            x2 =  -(D2.getDistance(DistanceUnit.INCH) - dwall2);
            y2 = ywall;
            o2 = Math.toRadians(180);

            x3 = dwall2 + dslam;
            y3 = ywall;
            o3 = o2;



        } else {
            x1 = (D4.getDistance(DistanceUnit.INCH) - dwall);
            y1 = 2;
            o1 = Math.toRadians(-90);

            x2 =  (D4.getDistance(DistanceUnit.INCH) - dwall2);
            y2 = ywall;
            o2 = Math.toRadians(0);

            x3 = dwall2 + dslam;
            y3 = ywall;
            o3 = o2;


        }
        drive.setPoseEstimate(new Pose2d(0,0,Math.toRadians(-90)));
        init1 = drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(x1,y1,o1))
                .splineToSplineHeading(new Pose2d(x2, y2, o2), o2)
                .build();

        drive.followTrajectorySequenceAsync(init1);
        drive.update();
        while( drive.isBusy()
                && !isStopRequested()){
            drive.update();
            Slide();
        }
    }

    public void Park() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        if (sidered) {
            if (zone == "1") {
                park = -58;
            }
            if (zone == "2") {
                park = -36;
            }
            if (zone == "3") {
                park = -12;
            }
        }else{
            if (zone == "1") {
                park = 12;
            }
            if (zone == "2") {
                park = 36;
            }
            if (zone == "3") {
                park = 58;
            }
        }

        if (sidered){
            vopark = Math.toRadians(0);
        }
        else{
            vopark = Math.toRadians(180);
        }
        drive.setPoseEstimate(prevpose);

        parktraj = drive.trajectorySequenceBuilder(prevpose)
                //.lineToLinearHeading(new Pose2d(vx,vy,vo))
                .back(2)
                .addDisplacementMarker(() ->{
                    target = 800;
                })
                .splineToSplineHeading(new Pose2d(park-.01,vy+3,vopark),vopark)
                .addDisplacementMarker(() ->{
                })
                .build();
        drive.followTrajectorySequenceAsync(parktraj);
        vx = park;
        vy = vy-3;
        while( drive.isBusy()
                && !isStopRequested()){
            drive.update();
            Slide();
        }
        autopose = drive.getPoseEstimate();
    }

    public void Cycle(){
        math();
        math();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        for(int i = 0;i < xcord.length; i++){

            xcordset = xm * xcord[i];
            ycordset = ycord[i];
            ServoClamp();
            Drive();
            if (i < xcord.length -1) {
                Drive();
                //Center();
                Slam();


            }
            else{
                drop();
            }
            /*telemetry.addData("vy",vy);
            telemetry.addData("iy", iy);
            telemetry.addData("x2", x2);
            telemetry.addData("y2",y2);
            telemetry.addData("o2", o2);
            telemetry.addData("x3", x3);
            telemetry.addData("y3", y3);
            telemetry.addData("o3", o3);
            telemetry.addData("distanceSensor", distance );

            telemetry.update();
             */

        }
    }
    public void Center(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());
        double distance = 0;
        double distanceholder = 0;
        int count = 0;
        if (sidered){

            tslam = drive.trajectorySequenceBuilder(new Pose2d())
                    .strafeLeft(100)
                    .build();

            drive.followTrajectorySequenceAsync(tslam);
            drive.update();
            while(D4.getDistance(DistanceUnit.INCH) > 300
                    && !isStopRequested()){
                drive.update();
                Slide();
            }
            /////////////////
            M1.setPower(0);
            M0.setPower(0);
            M2.setPower(0);
            M3.setPower(0);
            drive.setPoseEstimate(new Pose2d());
            for(int i = 0;i < 5; i++) {
                distanceholder = D4.getDistance(DistanceUnit.INCH);
                if (distanceholder < 55) {
                    distance += distanceholder;
                    count += 1;
                }
            }
            distance = distance/count;
            tslam = drive.trajectorySequenceBuilder(new Pose2d())
                    .strafeRight(centerpos-distance)
                    .build();

            drive.followTrajectorySequenceAsync(tslam);
            drive.update();
            while(drive.isBusy()
                    && !isStopRequested()){
                drive.update();
                Slide();
            }
        }else{

            tslam = drive.trajectorySequenceBuilder(new Pose2d())
                    .strafeRight(100)
                    .build();

            drive.followTrajectorySequenceAsync(tslam);
            drive.update();
            while(D2.getDistance(DistanceUnit.INCH) > 300
                    && !isStopRequested()){
                drive.update();
                Slide();
            }
            /////////////////
            M1.setPower(0);
            M0.setPower(0);
            M2.setPower(0);
            M3.setPower(0);
            drive.setPoseEstimate(new Pose2d());distance = 0;
            for(int i = 0;i < 5; i++) {
                distanceholder = D2.getDistance(DistanceUnit.INCH);
                if (distanceholder < 55) {
                    distance += distanceholder;
                    count += 1;
                }
            }
            distance = distance/count;

            telemetry.addData("distanceSensor", distance );
            telemetry.update();

            tslam = drive.trajectorySequenceBuilder(new Pose2d())
                    .strafeLeft(centerpos-distance)
                    .build();

            drive.followTrajectorySequenceAsync(tslam);
            drive.update();
            while(drive.isBusy()
                    && !isStopRequested()){
                drive.update();
                Slide();
            }

        }
    }

public void Slam(){
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    drive.setPoseEstimate(new Pose2d());
    tslam = drive.trajectorySequenceBuilder(new Pose2d())
            .forward(dslam)
            .build();

    drive.followTrajectorySequenceAsync(tslam);
    drive.update();
    while( drive.isBusy()
            && !isStopRequested()){
        drive.update();
        Slide();
    }
}
    public void ServoClamp() {
        S0.setPosition(0.05);
        M0_2.setPower(-.75);
        while (D5.getState() == false && M0_2.getCurrentPosition() > -150){
        }
        if (w == 1 || w ==4){
            S0.setPosition(.25);
        }
        else {
            S0.setPosition(.35);
        }
        //telemetry.addData("current",M0_2.getCurrentPosition());
        target = M0_2.getCurrentPosition() - bump;
        //telemetry.addData("target",target);
        //telemetry.update();
        UntilSlide();
        //S0.setPosition(.3);
        target = target + slideoffset;
        UntilSlide();
    }
    public void Drive() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        if(atwall)target = 850;
        else drop();
        math();
            drive.setPoseEstimate(currentpose);
        if (!atwall || translate) {
            traj = drive.trajectorySequenceBuilder(currentpose)
                    .lineToLinearHeading(new Pose2d(x1, y1, o1))
                    .addDisplacementMarker(() -> {
                        if (atwall) {
                            slidecalibrated = false;
                            target = starget;
                        }
                        if (!atwall) {
                            target = hdata[x + 5 * (y - 1) + 2];
                            if (hdata[xcordset + 5 * (ycordset - 1) + 2] > 1000) {
                                S1.setPosition(.7); //.02
                                S2.setPosition(.03);
                                ;//.7
                            }
                        }
                    })
                    .splineToSplineHeading(new Pose2d(x2, y2, o2), o2)
                    .addDisplacementMarker(() -> {
                        if (target < 150 && atwall == false) {  //if at ground station than drop cone and set slide up
                            S0.setPosition(0.05);
                            target = 800;
                        }
                    })
                    .splineToSplineHeading(new Pose2d(x3 - .01, y3 - .03, o3 + .01), o3)
                    .build();
        }else{
            traj = drive.trajectorySequenceBuilder(currentpose)
                    .back (d1-reverseoffset)
                    .addDisplacementMarker(() -> {
                        if (atwall) {
                            slidecalibrated = false;
                            target = starget;
                        }
                        if (!atwall) {
                            target = hdata[x + 5 * (y - 1) + 2];
                            if (hdata[xcordset + 5 * (ycordset - 1) + 2] > 1000) {
                                S1.setPosition(.7); //.02
                                S2.setPosition(.03);
                                ;//.7
                            }
                        }
                    })
                    .splineToSplineHeading(new Pose2d(x2, y2, o2), o2)
                    .addDisplacementMarker(() -> {
                        if (target < 150 && atwall == false) {  //if at ground station than drop cone and set slide up
                            S0.setPosition(0.05);
                            target = 800;
                        }
                    })
                    //.splineToSplineHeading(new Pose2d(x3 - .01, y3 - .01, o3 + .01), o3)
                    .build();
        }

        drive.followTrajectorySequenceAsync(traj);

        drive.update();
        while (Math.abs(gamepad1.left_stick_x) < .5
                && Math.abs(gamepad1.left_stick_y) < .5
                && Math.abs(gamepad1.right_stick_x) < .5
                && Math.abs(gamepad1.right_stick_y) < .5
                && drive.isBusy()
                && !isStopRequested()
                && Math.abs(gamepad1.right_trigger) < .5
                && Math.abs(gamepad1.left_trigger) < .5) {
            drive.update();
            Slide();
            prevpose = drive.getPoseEstimate();
        }
        if (!atwall) {
            preset += 1;
            if(preset <= xcord.length && useiteration){
                xcordset = xm * xcord[preset - 1];
                ycordset = ycord[preset - 1];
            }
            if (target > 500 && !beacon) {
                S0.setPosition(.25);
            }
            else{
                S0.setPosition(.05);
            }

        }

    }
    public void math() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        translate = true;
        y = ycordset;
        x = xcordset;
        w = wcordset;
        if (atwall) {
            if (y == 6) {
                d = d2;
            } else {
                d = d1;
            }
            if (w == 1) {
                if (y >= 3) {
                    vy = 24 * (y - 3) - 12;
                    if (x <= -2) {
                        vx = 24 * (x + 1) - 12;
                        vo = Math.toRadians(135);
                    } else {
                        vx = 24 * x - 12;
                        vo = Math.toRadians(45);
                    }
                } else {
                    vy = 24 * (y - 2) - 12;
                    if (x <= -2) {
                        vx = 24 * (x + 1) - 12;
                        vo = Math.toRadians(-135);
                    } else {
                        vx = 24 * x - 12;
                        vo = Math.toRadians(-45);
                    }
                }
                x1 = vx - offset;
                y1 = iy;
                o1 = io;
            }
            if (w == 2) {
                if (x >= 0) {
                    vx = 24 * x - 12;
                    if (y <= 1) {
                        vy = 24 * (y - 2) - 12;
                        vo = Math.toRadians(-45);
                    } else {
                        vy = 24 * (y - 3) - 12;
                        vo = Math.toRadians(45);
                    }
                } else {
                    vx = 24 * (x + 1) - 12;
                    if (y <= 1) {
                        vy = 24 * (y - 2) - 12;
                        vo = Math.toRadians(-135);
                    } else {
                        vy = 24 * (y - 3) - 12;
                        vo = Math.toRadians(135);
                    }
                }
                x1 = ix;
                y1 = vy - offset;
                o1 = io;
            }
            if (w == 3) {
                if (x >= 1) {
                    vx = 24 * (x - 1) + 12;
                    if (y <= 1) {
                        vy = 24 * (y - 2) - 12;
                        vo = Math.toRadians(-45);
                    } else {
                        vy = 24 * (y - 3) - 12;
                        vo = Math.toRadians(45);
                    }
                } else {
                    vx = 24 * x + 12;
                    if (y <= 1) {
                        vy = 24 * (y - 2) - 12;
                        vo = Math.toRadians(-135);
                    } else {
                        vy = 24 * (y - 3) - 12;
                        vo = Math.toRadians(135);
                    }
                }
                x1 = ix;
                y1 = vy - offset;
                o1 = io;
            }
            if (w == 4) {
                if (y >= 3) {
                    vy = 24 * (y - 3) - 12;
                    if (x >= 2) {
                        vx = 24 * (x-1 ) + 12;
                        vo = Math.toRadians(45);
                    } else {
                        vx = 24 * x + 12;
                        vo = Math.toRadians(135);
                    }
                } else {
                    vy = 24 * (y - 2) - 12;
                    if (x >= 2) {
                        vx = 24 * (x -1) + 12;
                        vo = Math.toRadians(-45);
                    } else {
                        vx = 24 * x + 12;
                        vo = Math.toRadians(-135);
                    }
                }
                x1 = vx + offset;
                y1 = iy;
                o1 = io;
            }
            if (y == 6) {
                vx = vx - 2 * xm;
            }
            x2 = vx + d * Math.cos(vo);
            y2 = vy + d * Math.sin(vo);
            o2 = vo;
            x3 =x2;
            y3 = y2;
            o3 = o2;
            if (usedistance) {
                double distance = 0;
                double distanceholder = 0;
                int count = 0;
                if (w == 1 || w == 3) {
                    for (int i = 0; i < 5; i++) {
                        distanceholder = D4.getDistance(DistanceUnit.INCH);
                        if (distanceholder < 55) {
                            distance += distanceholder;
                            count += 1;
                        }
                    }
                } else {
                    for (int i = 0; i < 5; i++) {
                        distanceholder = D2.getDistance(DistanceUnit.INCH);
                        if (distanceholder < 55) {
                            distance += distanceholder;
                            count += 1;
                        }
                    }
                }

                if (count == 0) {
                    distance = defaultcenter;
                } else {
                    distance = distance / count;
                }

                if (w == 1 || w ==4){
                    iy = distance - centerpos - 12;
                }else if (w == 2){
                    ix = distance - centerpos - 12;
                }else{
                    ix = centerpos - distance + 12;
                }
                telemetry.addData("distanceSensor", distance);
               // telemetry.update();
            }
            currentpose = new Pose2d(ix, iy, io);
            atwall = false;

        } else {
            x1 = vx + reverseoffset * Math.cos(vo);
            y1 = vy + reverseoffset * Math.sin(vo);
            o1 = vo;
            o2 = io;
            x3 = ix;
            y3 = iy;
            o3 = io;
            if (w == 1) {
                ix = -64;
                iy = -12;
                io = Math.toRadians(180);
                starget = 850;
                y2 = iy;
                x2 = vx - offset;
                if (vy == iy){
                    x2 = x3;
                    y2 = y3;
                    o2 = o3;
                    translate = false;
                }
            }
            if (w == 2) {
                ix = -12;
                iy = -64;
                io = Math.toRadians(-90);
                starget = 500;
                y2 = vy - offset;
                x2 = ix;
                if (vx == ix){
                    x2 = x3;
                    y2 = y3;
                    o2 = o3;
                    translate = false;
                }
            }
            if (w == 3) {
                ix = 12;
                iy = -64;
                io = Math.toRadians(-90);
                starget = 500;
                y2 = vy - offset;
                x2 = ix;
                if (vx == ix){
                    x2 = x3;
                    y2 = y3;
                    o2 = o3;
                    translate = false;
                }

            }
            if (w == 4) {
                ix = 64;
                iy = -12;
                io = 0;
                starget = 850;
                y2 = iy;
                x2 = vx + offset;
                if (vy == iy){
                    x2 = x3;
                    y2 = y3;
                    o2 = o3;
                    translate = false;
                }
            }




            currentpose = prevpose;//new Pose2d(vx + d * Math.cos(vo), vy + d * Math.sin(vo), vo);
            if (!beacon){
                atwall = true;
            }
        }
    }


    public void IdentifyVuforia(){
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Objects Detected", updatedRecognitions.size());
                // step through the list of recognitions and display image position/size information for each one
                // Note: "Image number" refers to the randomized image orientation/number
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData("GoTo", recognition.getLabel(), recognition.getConfidence() * 100 );
                    zone = recognition.getLabel();
                }
                telemetry.addData("zone",zone);
                //telemetry.addLine("working");
                telemetry.addData("sidered",sidered);
                telemetry.addData("", "");
                telemetry.addData("right", D2.getDistance(DistanceUnit.INCH));
                telemetry.addData("left", D4.getDistance(DistanceUnit.INCH));


                //telemetry.addData("xm",xm);
                telemetry.update();
            }
        }
    }
    public void Slide () {

        if (slidecalibrated) {
            M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - 1.4*M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - 1.4*M0_2.getCurrentPosition()) / 250)))));
        }

        else {

                if (D0.getState() == true ) {
                    M0_2.setPower(.3);
                    beenoff = true;
                }
                if (D0.getState() == false  && !beenoff) {
                    M0_2.setPower(-calibratespeed);
                    /*if (M0_2.getCurrentPosition() > 100){
                        M0_2.setPower(-1 * ((1 - Math.pow(10, ((-1.4*M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((-1.4*M0_2.getCurrentPosition()) / 250)))));
                    }else{
                        M0_2.setPower(-1 * ((1 - Math.pow(10, ((-140) / 250))) / (1 + Math.pow(10, ((-140) / 250)))));

                    }*/


                }
                if (D0.getState() == false && beenoff) { //if slide is on limit and calibratedM0_2.setDirection(DcMotor.Direction.FORWARD);
                    M0_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    M0_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    M0_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    slidecalibrated = true;
                    beenoff = false;
                    M0_2.setPower(0);
                //}
            }
        }
    }
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.1f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 330;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
    public void UntilSlide() {
        if ((target - 1.4*M0_2.getCurrentPosition()) > 0) {
            M0_2.setPower(slidespeed);
            while (target > 1.4*M0_2.getCurrentPosition()) { //faster slide algo
                //* ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition())))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()))))));
            }
        }
        else{
            M0_2.setPower(-slidespeed );
            while (target < 1.4*M0_2.getCurrentPosition()) { //faster slide algo
                //* ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition())))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()))))));
            }
        }
        M0_2.setPower(0);
    }
    public void drop(){
        if (beacon){
            S0.setPosition(0.25);
        }
        else {
            S0.setPosition(0.05);
        }
        double pt = target;
        target = target - Sdrop/2;
        UntilSlide();
        S1.setPosition(0.02); //.02
        S2.setPosition(.7); ;//.7
        target = target - Sdrop/2;
        UntilSlide();
        target = pt;
        UntilSlide();
    }
}



