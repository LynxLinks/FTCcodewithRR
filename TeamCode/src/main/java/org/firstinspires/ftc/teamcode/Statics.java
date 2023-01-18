package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.teamcode.TestServos.UmbrellaMax1;
import static org.firstinspires.ftc.teamcode.TestServos.UmbrellaMax2;
import static org.firstinspires.ftc.teamcode.TestServos.UmbrellaMin1;
import static org.firstinspires.ftc.teamcode.TestServos.UmbrellaMin2;
import static org.firstinspires.ftc.teamcode.TestServos.camBothClosed;
import static org.firstinspires.ftc.teamcode.TestServos.camBothOpen;
import static org.firstinspires.ftc.teamcode.TestServos.camTopOpen;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

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


    public static double dslam = 2;//1.5
    public static double d2 = 3;
    public static double centerposblue = 59;//53.5
    public static double centerposred = 59;//53.5
    public static double offset1 = 12;
    public static double offset2 = 5;
    public static double offset3 = 10;
    public static double offset4 = 2;
    public static double offset5 = 9;
    public static double offset6 = 13;
    public static double Sdrop = 450;
    public static double slidespeed = .6;
    public static double bump = 250;
    public static double calibratespeed = 1;
    public static double vopark;

    int[] hdata = {150, 1000, 150, 1000, 150,
    1000, 1770, 2329, 1770, 1000,
    150, 2329, 150, 2329, 150,
    1000, 1770, 2329, 1770, 1000,
    150, 1000, 150, 1000, 150
    ,100,100,100,100,100,100,100,100,100,100,100};

    int [][][] odata = {
    {
            {-120,-91,91,120,110},
            {-60,-38,38,60,70},
            {-60,-38,38,60,70},
            {-60,-38,38,60,70},
            {-60,-38,38,60,70},
    },{
            {120,60,60,60,60},
            {91,38,38,38,38},
            {-91,-38,-38,-38,-38},
            {-120,-60,-60,-60,-60},
            {-110,-70,-70,-70,-70},
    },{
            {110,70,70,70,70},
            {120,60,60,60,60},
            {91,38,38,38,38},
            {-91,-38,-38,-38,-38},
            {-120,-60,-60,-60,-60},

    },{
            {60,38,-38,-60,-70},
            {60,38,-38,-60,-70},
            {60,38,-38,-60,-70},
            {60,38,-38,-60,-70},
            {120,91,-91,-60,-110},
    },
    };
    int [] ordata = {0,90,90,180};
    double starget;
    double reverseoffset;
    double d;
    double ix;
    double iy;
    double io;
    double cx;
    double cy;
    double x1;
    double x2;
    double x3;
    double y1;
    double y2;
    double y3;
    double o1;
    double o2;
    double o3;
    double target;
    double b1;
    double vy;
    double vx;
    double angdelta;
    double turn;
    double vo;
    double distance = 0;
    double slideoffset;
    double offset;
    double centerpos;
    double vopark2;
    double o;
    int xcordset;
    int zonei;
    int ycordset = 2;
    int wcordset;
    int y = 2;
    int x;
    int w;
    int initw;
    int initx;
    int inity;
    int position;
    int xm;
    boolean beacon;
    boolean righttrig;
    boolean auto;
    boolean angleactive;
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
    boolean initialmovement = true;
    boolean atwall = true;
    Pose2d currentpose;
    Pose2d startpose;
    TrajectorySequence traj;
    TrajectorySequence init1;
    SampleMecanumDrive drive;
    public void runOpMode() {}
    public void StaticInit(boolean autof,double d1f,double slideoffsetf, double reverseoffsetf,double offsetf){
        reverseoffset = reverseoffsetf;
        slideoffset = slideoffsetf;
        offset = offsetf;
        auto = autof;
        d = d1f;
        if (!auto){
            wcordset = (PoseStorage.initw);
            xcordset = (PoseStorage.initx);
            ycordset = (PoseStorage.inity);
        }

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
        S0.setPosition(camBothClosed);
        S1.setPosition(UmbrellaMin1); //.7
        S2.setPosition(UmbrellaMax2); //.03
    }
    public void rrinnit(){
        drive = new SampleMecanumDrive(hardwareMap);
    }
    public void UntilSlide() {
        telemetry.addLine("untilslide");
        telemetry.update();
        if ((target > 1.4*M0_2.getCurrentPosition())) {
            M0_2.setPower(slidespeed);
            while (target > 1.4*M0_2.getCurrentPosition() && gamepad2.right_trigger < .6 && !gamepad1.b){
                if (!auto){
                    manual();
                    UI();
                }else{
                    drive.update();
                }
            }
        }
        else{
            M0_2.setPower(-slidespeed );
            while (target < 1.4*M0_2.getCurrentPosition() && gamepad2.right_trigger < .6  && !gamepad1.b) {
                if (!auto){
                    manual();
                    UI();
                }else{
                    drive.update();
                }
            }
        }
        M0_2.setPower(0);
    }
    public void drivestack(){
        drive.update();
        drive.update();
        drive.update();
        drive.update();
        drive.update();
    }
    public void drop(){
        if (beacon){
            S0.setPosition(camTopOpen);
        }
        else {
            S0.setPosition(camBothClosed);
        }
        if (target > 500) {
            double pt = target;
            target = target - Sdrop;
            UntilSlide();
            S1.setPosition(UmbrellaMin1); //.7
            S2.setPosition(UmbrellaMax2); //.03
            target = pt;
            UntilSlide();
            if (beacon){
                S1.setPosition(UmbrellaMax1); //.7
                S2.setPosition(UmbrellaMin2); //.03
            }
        }
        beacon = false;
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
                if (1.4*M0_2.getCurrentPosition()>500){
                    M0_2.setPower(-calibratespeed);
                }else{
                    M0_2.setPower(-.2);
                }
            }
            if (D0.getState() == false && beenoff) { //if slide is on limit and calibratedM0_2.setDirection(DcMotor.Direction.FORWARD);
                M0_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                M0_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                M0_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slidecalibrated = true;
                beenoff = false;
            }
        }
    }
    public void manual(){
        drive.updatePoseEstimate();
        if (initialmovement){
            o = drive.getPoseEstimate().getHeading();
            initialmovement = false;
            math2(xcordset,ycordset,wcordset,false);
            math2(xcordset,ycordset,wcordset,false);
        }
        if (Math.abs(gamepad1.right_stick_y) + Math.abs(gamepad1.right_stick_x) > .5) {
            angleactive = true;
            o = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x);
            o = o + Math.toRadians(180) * (1 - o / Math.abs(o));
            if (o != o){
                o = 0;
            }
        }

        angdelta = (Math.toDegrees(o - drive.getPoseEstimate().getHeading()));
        if (angdelta > 180){
            angdelta -= 360;
        }else if (angdelta < -180){
            angdelta += 360;
        }

        if (gamepad1.left_trigger > .4 || gamepad1.right_trigger > .4 ){
            turn = 0.2*gamepad1.left_trigger - 0.2*gamepad1.right_trigger;
            o = drive.getPoseEstimate().getHeading();
        }else{
            if (angleactive){
                turn = -.6* Math.pow(((1 - Math.pow(9, ((angdelta) / 14))) / (1 + Math.pow(9, ((angdelta) / 14)))),3);// - gamepad1.right_trigger * .2;

            }else{
                turn = 0;
                o = drive.getPoseEstimate().getHeading();
            }
        }
        double sm = .65;
        if (gamepad1.left_stick_button){
            sm = .18;
        }
        Vector2d input = new Vector2d(
                gamepad1.left_stick_x*sm,//+ gamepad1.right_stick_x*sm,
                -gamepad1.left_stick_y*sm
                // - gamepad1.right_stick_y*sm
        ).rotated(-drive.getPoseEstimate().getHeading());

        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        turn
                )
        );
    }

    public void ServoClamp() {
        telemetry.addLine("ServoClamp");
        telemetry.update();
        angleactive = false;
        S0.setPosition(camBothClosed);
        M0_2.setPower(-.75);
        while (D5.getState() == false && M0_2.getCurrentPosition()*1.4 > 0) {
            if (!auto){
                manual();
                UI();
            }
        }
        if (wcordset == 1 || wcordset == 4) {
            S0.setPosition(camTopOpen);
        } else {
            S0.setPosition(camBothOpen);
        }

        target = M0_2.getCurrentPosition() * 1.4 - bump;
        UntilSlide();
        target = target + slideoffset;
        UntilSlide();
        if(!D5.getState()){
            S0.setPosition(camBothClosed);
        }
    }
    public void UI() {

        if (gamepad1.a) {
            target = starget;
            S1.setPosition(UmbrellaMin1); //.7
            S2.setPosition(UmbrellaMax2); //.03
        }
        if (gamepad1.b) {
            target = hdata[7];
            S1.setPosition(UmbrellaMax1); //.7
            S2.setPosition(UmbrellaMin2); //.03
        }
        if (gamepad1.y) {
            target = hdata[6];
            S1.setPosition(UmbrellaMax1); //.7
            S2.setPosition(UmbrellaMin2); //.03
        }
        if (gamepad1.x){
            target = hdata[5];
            S1.setPosition(UmbrellaMin1); //.7
            S2.setPosition(UmbrellaMax2); //.03
        }
        if (gamepad2.left_stick_button){ //down
            S1.setPosition(UmbrellaMax1); //.7
            S2.setPosition(UmbrellaMin2); //.03
        }if (gamepad2.right_stick_button){ // up
            S1.setPosition(UmbrellaMin1); //.02
            S2.setPosition(UmbrellaMax2); ;//.7
        }
        if (!gamepad2.dpad_up) dup = true;
        if (!gamepad2.dpad_down) ddown = true;
        if (!gamepad2.dpad_left) dleft = true;
        if (!gamepad2.dpad_right) dright = true;
        if (!gamepad2.right_bumper) dbright = true;
        if (!gamepad2.left_bumper) dbleft = true;
        if (!gamepad1.dpad_right) dright2 = true;
        if (!gamepad1.dpad_left) dleft2 = true;
        if (gamepad2.right_bumper && wcordset < 4 && dbright) {
            dbright = false;
            wcordset += 1;
        }
        if (gamepad2.left_bumper && wcordset > 1 && dbleft) {
            dbleft = false;
            wcordset -= 1;
        }
        if ((gamepad2.dpad_up) && dup && ycordset < 5) {
            dup = false;
            ycordset += 1;
        }
        if ((gamepad2.dpad_down) && ddown && ycordset > 1) {
            ddown = false;
            ycordset -= 1;
        }
        if ((gamepad2.dpad_right) && dright&& xcordset < 2) {
            dright = false;
            xcordset += 1;
        }
        if ((gamepad2.dpad_left) && dleft && xcordset > -2) {
            dleft = false;
            xcordset -= 1;
        }
        if(gamepad2.right_stick_y > .5){
            beacon = true;
        }
        if(gamepad2.right_stick_y < -.5){
            beacon = false;
        }
        if(gamepad2.a){
            xcordset = 0;
            ycordset = 2;
        }
        if (gamepad1.dpad_up){
            S1.setPosition(UmbrellaMin1); //.02
            S2.setPosition(UmbrellaMax2); ;//.7
        }
        if (gamepad1.dpad_down){
            S1.setPosition(UmbrellaMax1); //.7
            S2.setPosition(UmbrellaMin2); //.03
        }
        if((gamepad1.dpad_right) && dright2){
            dright2 = false;
            Drive(xcordset,ycordset,wcordset,false);
        }
        if (gamepad1.right_stick_button){
            drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY(),Math.toRadians(-90)));
        }
        if((gamepad1.dpad_left) && dleft2){
            dleft2 = false;
            Drive(xcordset,ycordset,wcordset,true);
        }
        if (gamepad2.right_trigger < 0.6) righttrig = true;
        if(gamepad2.right_trigger > 0.6 && righttrig){
            righttrig = false;
            if (slidecalibrated == false) {
                slidecalibrated = true;
            }
            else if (slidecalibrated == true) {
                slidecalibrated = false;
            }
        }
        if(!gamepad2.b) Bbutton = true;
        if(gamepad2.b && Bbutton){
            Bbutton = false;
            math2(xcordset,ycordset,wcordset,false);
        }
        telemetry.addData("x", xcordset);
        telemetry.addData("", "");
        telemetry.addData("y", ycordset);
        telemetry.addData("", "");
        telemetry.addData("w", wcordset);
        telemetry.addData("", "");
        telemetry.addData("atwall", atwall);
        telemetry.addData("", "");
        telemetry.addData("beacon", beacon);
        telemetry.addData("angdleta", angdelta);
        telemetry.update();
    }
    public void Drive(int xf, int yf, int wf, boolean savepos) {

        telemetry.addLine("Drive");
        telemetry.update();

        double dslamf = 0;
        if (atwall) {

            target = 850;
        } else {
            if (auto){
                dslamf = dslam;
            }
            if (target < 1000){
                target = 400;
            }else {
                drop();
            }
        }
        math2(xf, yf, wf,savepos);
        drive.setPoseEstimate(currentpose);
        traj = drive.trajectorySequenceBuilder(startpose)
                .back(b1)
                .addDisplacementMarker(() -> {
                    if (atwall) {
                        target = starget;
                    }else{
                        target = hdata[xf + 5 * (yf - 1) + 2];
                        if (hdata[xf + 5 * (yf - 1) + 2] > 1400) {
                            S1.setPosition(UmbrellaMax1); //.7
                            S2.setPosition(UmbrellaMin2); //.03
                        }
                        if (target < 150) {  //if at ground station than drop cone and set slide up
                            S0.setPosition(camBothClosed);
                        }
                    }
                })
                .splineToSplineHeading(new Pose2d(x2 + dslamf*(x2/Math.abs(x2)), y2 , o2), o2)
                .splineToSplineHeading(new Pose2d(x3+ dslamf*(x2/Math.abs(x2)) +.01 , y3+ .01 , o3 ), o3)
                .build();
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
            drivestack();
            Slide();
            if (!auto) {
                UI();
            }
        }
        if ((target > 2000 || target < 600)&& !beacon && !atwall) {
            S0.setPosition(camBothClosed);
        }
        o = drive.getPoseEstimate().getHeading();
    }
    public void math2(int xf,int yf, int wf,boolean savepos) {
        telemetry.addLine("Math");
        telemetry.update();
        y = yf;
        x = xf;
        w = wf;
        if (atwall) {
            vy = (y - 3) * 24;
            vx = x * 24;
            vo = Math.toRadians(odata[wf - 1][xf + 2][yf - 1] + ordata[w - 1]);
            x2 = vx + d * Math.cos(vo);
            y2 = vy + d * Math.sin(vo);
            o2 = vo;
            x3 = x2;
            y3 = y2;
            o3 = o2;
            cx = Math.round((vx - d * Math.cos(vo) + 12) / 24) * 24 - 12;
            cy = Math.round((vy - d * Math.sin(vo) + 12) / 24) * 24 - 12;

            if (w == 1) {
                if (Math.abs(iy - cy) == 0) {
                    offset = offset1;
                }
                if (Math.abs(iy - cy) == 24) {
                    offset = offset2;
                }
                if (Math.abs(iy - cy) == 48) {
                    offset = offset3;

                }
                b1 = cx - ix - offset;
            }
            if (w == 2 || w == 3) {
                if (Math.abs(ix - cx) == 0) {
                    offset = offset1;
                }
                if (Math.abs(ix - cx) == 24) {
                    offset = offset2;
                }
                if (Math.abs(ix - cx) == 48) {
                    offset = offset3;

                }
                b1 = cy - iy - offset;
            }
            if (w == 4) {
                if (Math.abs(iy - cy) == 0) {
                    offset = offset1;
                }
                if (Math.abs(iy - cy) == 24) {
                    offset = offset2;
                }
                if (Math.abs(iy - cy) == 48) {
                    offset = offset3;

                }
                b1 = ix - cx - offset;
            }
            x2 = vx - d * Math.cos(vo);
            y2 = vy - d * Math.sin(vo);
            o2 = vo;
            x3 = x2;
            y3 = y2;
            o3 = o2;

            if (savepos && auto && drive.getPoseEstimate().getY() > -24 && drive.getPoseEstimate().getY() < 0) {
                currentpose = new Pose2d(ix, drive.getPoseEstimate().getY(), io);
                startpose = currentpose;//changed

            } else if (savepos && !auto){
                currentpose = drive.getPoseEstimate();
                startpose = new Pose2d(ix, iy, io);
            } else {
                currentpose = new Pose2d(ix, iy, io);
                startpose = currentpose;

            }

            // }
            atwall = false;

        } else {
            if (w == 1) {
                ix = -64;
                iy = -12;
                io = Math.toRadians(180);

                starget = 850;

                y2 = iy;

                x2 = cx - offset;
                if (Math.abs(iy - cy) == 0) {
                    offset = offset4;
                }
                if (Math.abs(iy - cy) == 24) {
                    offset = offset5;
                }
                if (Math.abs(iy - cy) == 48) {
                    offset = offset6;

                }

            }
            if (w == 2) {
                if (Math.abs(ix - cx) == 0) {
                    offset = offset4;
                }
                if (Math.abs(ix - cx) == 24) {
                    offset = offset5;
                }
                if (Math.abs(ix - cx) == 48) {
                    offset = offset6;
                }
                ix = -12;
                iy = -64;
                io = Math.toRadians(-90);
                starget = 500;
                y2 = cy - offset;
                x2 = ix;
            }
            if (w == 3) {
                if (Math.abs(ix - cx) == 0) {
                    offset = offset4;
                }
                if (Math.abs(ix - cx) == 24) {
                    offset = offset5;
                }
                if (Math.abs(ix - cx) == 48) {
                    offset = offset6;
                }
                ix = 12;
                iy = -64;
                io = Math.toRadians(-90);
                starget = 500;
                y2 = cy - offset;
                x2 = ix;
            }
            if (w == 4) {
                ix = 64;
                iy = -12;
                io = 0;
                starget = 850;
                y2 = iy;
                x2 = cx + offset;
                if (Math.abs(iy - cy) == 0) {
                    offset = offset4;
                }
                if (Math.abs(iy - cy) == 24) {
                    offset = offset5;
                }
                if (Math.abs(iy - cy) == 48) {
                    offset = offset6;
                }
            }
            b1 = reverseoffset;
            o1 = vo;
            o2 = io;
            x3 = ix;
            y3 = iy;
            o3 = io;
            if (w == 4 || w == 1) {
                if (cy == iy) {
                    x2 = x3;
                    y2 = y3;
                    o2 = o3;
                }
            } else {
                if (cx == ix) {
                    x2 = x3;
                    y2 = y3;
                    o2 = o3;
                }
            }
            atwall = true;
            if (savepos) {
                currentpose = drive.getPoseEstimate();
            } else {
                currentpose = new Pose2d(vx + d * Math.cos(vo), vy + d * Math.sin(vo), (((Math.round((Math.toDegrees(vo) - 45)/90))*90)+45)   );
            }
            if (!beacon) {
                atwall = true;
            }
            startpose = currentpose;
            }
        }
    }
