package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.google.gson.annotations.Until;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.teamcode.TestServos.UmbrellaMax1;
import static org.firstinspires.ftc.teamcode.TestServos.UmbrellaMax2;
import static org.firstinspires.ftc.teamcode.TestServos.UmbrellaMin1;
import static org.firstinspires.ftc.teamcode.TestServos.UmbrellaMin2;
import static org.firstinspires.ftc.teamcode.DriveV10.useiteration;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;

@Config

public class Statics extends LinearOpMode {
    FtcDashboard dashboard;
    DcMotor M0;
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
    TrajectorySequence traj;
    Pose2d currentpose;
    Pose2d prevpose;



    //auto vars


    //teleop vars

    //common vars

    public static double d2 = 3;
    public static double centerpos = 50.2;
    public static double stagger = .8;
    public static double defaultcenter = 52;
    public static double offset = 12;
    public static double reverseoffset = 8;
    public static double Sdrop = 450;
    public static double slideoffset = 100;
    public static double slidespeed = .6;
    public static double bump = 150;
    public static double calibratespeed = 1;
    public static double vopark;



    int[] hdata = {100, 1150, 100, 1150, 100,
    1150, 1750, 2275, 1750, 1150,
    100, 2275, 100, 2275, 100,
    1150, 1750, 2275, 1750, 1150,
    100, 1150, 100, 1150, 100
    ,200,200,200,200,200,200,200,200,200,200,200};

    TrajectorySequence init1;
    TrajectorySequence parktraj;
    boolean translate;
    int[] xcord;
    int[] ycord;
    int position;
    int preset;
    int xm;
    boolean gx;
    boolean gy;
    double target;
    boolean beacon;
    boolean auto;
    double park;
    int xcordset;
    int ycordset;
    int wcordset;
    double vy;
    double vx;
    double vo;
    double d1;
    int y;
    int x;
    int w;
    boolean dup;
    boolean ddown;
    boolean dright;
    boolean dleft;
    boolean dbright;
    boolean dleft2;
    boolean dright2;
    boolean dbleft;
    boolean slidecalibrated;
    boolean beenoff;
    boolean Bbutton;
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
    boolean usedistance = true;



    public void runOpMode() {


    }
    public void StaticInit(boolean autof,double d1f, int[] xcordf, int[]ycordf, boolean useiterationf){
        auto = autof;
        d1 = d1f;
        xcord = xcordf;
        ycord = ycordf;
        useiteration = useiterationf;
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
        target = M0_2.getCurrentPosition() - bump;
        UntilSlide();
        target = target + slideoffset;
        UntilSlide();
    }
    public void Drive(int xf, int yf, int wf) {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        if(atwall)target = 850;
        else drop();
        math(xf,yf,wf);
        drive.setPoseEstimate(currentpose);
        if (!atwall){ //|| translate) {changed
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
                                S1.setPosition(UmbrellaMax1); //.7
                                S2.setPosition(UmbrellaMin2); //.03
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
                        slidecalibrated = false;
                        target = starget;
                    })
                    .splineToSplineHeading(new Pose2d(x2, y2-stagger, o2), o2)
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
    public void math(int xf,int yf, int wf) {

        translate = true;
        y = yf;
        x = xf;
        w = wf;
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
                double distanceholder;
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
                telemetry.update();

            }
            currentpose = new Pose2d(ix, iy, io);
            atwall = false;

        } else {
            if (w == 1) {
                ix = -64;
                iy = -12;
                io = Math.toRadians(180);
                starget = 850;
                y2 = iy;
                x2 = vx - offset;

            }
            if (w == 2) {
                ix = -12;
                iy = -64;
                io = Math.toRadians(-90);
                starget = 500;
                y2 = vy - offset;
                x2 = ix;

            }
            if (w == 3) {
                ix = 12;
                iy = -64;
                io = Math.toRadians(-90);
                starget = 500;
                y2 = vy - offset;
                x2 = ix;

            }
            if (w == 4) {
                ix = 64;
                iy = -12;
                io = 0;
                starget = 850;
                y2 = iy;
                x2 = vx + offset;

            }
            x1 = vx + reverseoffset * Math.cos(vo);
            y1 = vy + reverseoffset * Math.sin(vo);
            o1 = vo;
            o2 = io;
            x3 = ix;
            y3 = iy;
            o3 = io;

            if ( w == 4 || w == 1) {
                if (vy == iy) {
                    x2 = x3;
                    y2 = y3;
                    o2 = o3;
                    translate = false;
                }
            }else{
                if (vx == ix){
                    x2 = x3;
                    y2 = y3;
                    o2 = o3;
                    translate = false;
                }
            }
            if (auto) {
                currentpose = prevpose;
            }else{
                currentpose = new Pose2d(vx + d * Math.cos(vo), vy + d * Math.sin(vo), vo);
            }
            if (!beacon){
                atwall = true;
            }
        }
    }
    public void UntilSlide() {
        if ((target - 1.4*M0_2.getCurrentPosition()) > 0) {
            M0_2.setPower(slidespeed);
            while (target > 1.4*M0_2.getCurrentPosition());
        }
        else{
            M0_2.setPower(-slidespeed );
            while (target < 1.4*M0_2.getCurrentPosition()) ;
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
        target = target - Sdrop;
        UntilSlide();
        S1.setPosition(UmbrellaMin1); //.7
        S2.setPosition(UmbrellaMax2); //.03
        target = pt;
        UntilSlide();
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
            }
            if (D0.getState() == false && beenoff) { //if slide is on limit and calibratedM0_2.setDirection(DcMotor.Direction.FORWARD);
                M0_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                M0_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                M0_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                slidecalibrated = true;
                beenoff = false;
                M0_2.setPower(0);

            }
        }
    }

}