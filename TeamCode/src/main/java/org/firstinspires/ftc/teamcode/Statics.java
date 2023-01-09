package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

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
import static org.firstinspires.ftc.teamcode.DriveV10.useiteration;
import static org.firstinspires.ftc.teamcode.Auto14.stagger;
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
    TrajectorySequence traj;
    Pose2d currentpose;
    Pose2d prevpose;

    public static double dslam = 2.5;//1.5
    public static double d2 = 3;
    public static double centerposblue = 51;//53.5
    public static double centerposred = 64;//53.5
    public static double centerposoffset = -1;
    //public static double defaultcenter = 51;
    public static double offset = 24;

    public static double Sdrop = 450;
    public static double slidespeed = .6;
    public static double bump = 250;
    public static double calibratespeed = 1;
    public static double vopark;

    boolean broke = false;

    int[] hdata = {50, 1150, 50, 1150, 50,
    1150, 1775, 2300, 1775, 1150,
    50, 2300, 50, 2300, 50,
    1150, 1775, 2300, 1775, 1150,
    50, 1150, 50, 1150, 50
    ,200,200,200,200,200,200,200,200,200,200,200};

    double starget;
    double reverseoffset;
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
    double x4;
    double y4;
    double o4;
    double target;
    double b1;
    double vy;
    double vx;
    double vo;
    double d1;
    double distance = 0;
    double centerpos;
    double park;
    double slideoffset;
    boolean centered;
    int xcordset;
    int ycordset;
    int wcordset;
    int y;
    int x;
    int w;
    int[] xcord;
    int[] ycord;
    int position;
    int preset;
    int xm;
    boolean translate;
    boolean gx;
    boolean gy;
    boolean beacon;
    boolean auto;
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
    boolean atwall = true;
    //boolean usedistance = true;
    boolean servoclamp1 = true;
    boolean servoclamp2;
    boolean servoclamp3;
    boolean servoclamp4;
    boolean servoclampasync;
    boolean inited;
    TrajectorySequence init1;
    TrajectorySequence init2;
    TrajectorySequence parktraj;
    SampleMecanumDrive drive;

    public void runOpMode() {}
    public void StaticInit(boolean autof,double d1f, int[] xcordf, int[]ycordf, boolean useiterationf,double slideoffsetf, int position, double reverseoffsetf){
        if (autof){
            if ((position == 1|| position == 2) && !inited){
                centerposred = centerposred + centerposoffset;
                centerposblue = centerposblue + centerposoffset;
                inited = true;
            }
        }else{
            stagger = 0;
        }
        reverseoffset = reverseoffsetf;
        slideoffset = slideoffsetf;
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
    public void rrinnit(){
        drive = new SampleMecanumDrive(hardwareMap);
    }
    public void UntilSlide() {
        if ((target > 1.4*M0_2.getCurrentPosition())) {
            M0_2.setPower(slidespeed);
            while (target > 1.4*M0_2.getCurrentPosition()){
                if (!auto){
                    manual();
                    UI();
                }
            }
        }
        else{
            M0_2.setPower(-slidespeed );
            while (target < 1.4*M0_2.getCurrentPosition()) {
                if (!auto){
                    manual();
                    UI();
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
        drive.update();
    }
    public void drop(){
        if (beacon){
            S0.setPosition(camTopOpen);
        }
        else {
            S0.setPosition(camBothClosed);
        }
        double pt = target-(Sdrop/2);
        target = target - Sdrop;
        UntilSlide();
        S1.setPosition(UmbrellaMin1); //.7
        S2.setPosition(UmbrellaMax2); //.03
        target = pt;
        UntilSlide();

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
                if (1.4*M0_2.getCurrentPosition()>300){
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
                //M0_2.setPower(0);

            }
        }
    }
    public void manual(){

        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y*.5 - 0.2*gamepad1.right_stick_y,
                        -gamepad1.left_stick_x*.5- 0.2*gamepad1.right_stick_x,
                        gamepad1.left_trigger*.5 - gamepad1.right_trigger*.5
                )
        );

    }
    public void ServoClamp() {
        S0.setPosition(camBothClosed);
        M0_2.setPower(-.75);
        while (D5.getState() == false && M0_2.getCurrentPosition()*1.4 > -150) {
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

    }
    public void  ServoClampAsync() {
        if (servoclampasync) {
            if (servoclamp1) {
                S0.setPosition(camBothClosed);
                M0_2.setPower(-.75);
                servoclamp1 = false;
                servoclamp2 = true;
            } else if (servoclamp2) {
                if ((D5.getState() == true || M0_2.getCurrentPosition() < -150)) {
                    if (w == 1 || w == 4) S0.setPosition(camTopOpen);
                    else S0.setPosition(camBothOpen);
                    M0_2.setPower(-slidespeed);
                    target = M0_2.getCurrentPosition() * 1.4 - bump;
                    servoclamp2 = false;
                    servoclamp3 = true;
                }
            } else if (servoclamp3) {
                if (M0_2.getCurrentPosition() * 1.4 < target) {
                    M0_2.setPower(slidespeed);
                    target = target + slideoffset;
                    servoclamp3 = false;
                    servoclamp4 = true;
                }
            } else if (servoclamp4) {
                if (M0_2.getCurrentPosition() * 1.4 > target) {
                    M0_2.setPower(.01);
                    servoclamp4 = false;
                    servoclamp1 = true;
                    servoclampasync = false;
                }
            }

        }
    }
    public void UI() {


        //Manual Servo
        if (gamepad1.left_bumper) {
            S0.setPosition(camBothClosed);
        }

        //Manual Slide
        if (gamepad1.a) target = starget;
        if (gamepad1.b) target = hdata[7];
        if (gamepad1.y) target = hdata[6];
        if (gamepad1.x) target = hdata[5];

        //Mauanl Umbrella
        if (gamepad2.left_stick_button){ //down
            S1.setPosition(UmbrellaMax1); //.7
            S2.setPosition(UmbrellaMin2); //.03
        }if (gamepad2.right_stick_button){ // up
            S1.setPosition(UmbrellaMin1); //.02
            S2.setPosition(UmbrellaMax2); ;//.7
        }

        //coordinates
        if (!gamepad2.dpad_up) dup = true;
        if (!gamepad2.dpad_down) ddown = true;
        if (!gamepad2.dpad_left) dleft = true;
        if (!gamepad2.dpad_right) dright = true;
        if (!gamepad2.right_bumper) dbright = true;
        if (!gamepad2.left_bumper) dbleft = true;
        if (!gamepad1.dpad_right) dright2 = true;
        if (!gamepad1.dpad_left) dleft2 = true;

        if (gamepad2.x && preset < xcord.length) gx = true;
        if (gamepad2.y && preset > 1) gy = true;
        if (gamepad2.right_bumper && wcordset < 4 && dbright) {
            dbright = false;
            wcordset += 1;
        }
        if (gamepad2.left_bumper && wcordset > 1 && dbleft) {
            dbleft = false;
            wcordset -= 1;
        }
        if ((gamepad2.dpad_up) && dup && wcordset < 4) {
            dup = false;
            ycordset += 1;
        }
        if ((gamepad2.dpad_down) && ddown) {
            ddown = false;
            ycordset -= 1;
        }
        if ((gamepad2.dpad_right) && dright) {
            dright = false;
            xcordset += 1;
        }
        if ((gamepad2.dpad_left) && dleft) {
            dleft = false;
            xcordset -= 1;

        }
        if((gamepad2.x) && gx){
            preset += 1;
            xcordset = xm*xcord[preset-1];
            ycordset = ycord[preset-1];

            gx = false;
        }
        if((gamepad2.y) && gy){
            preset -= 1;
            xcordset = xm*xcord[preset-1];
            ycordset = ycord[preset-1];

            gy = false;
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
        if((gamepad1.dpad_right) && dright2){
            dright2 = false;
            //usedistance = true;
            Drive(xcordset,ycordset,wcordset,true,true,0,0);

        }
        if((gamepad1.dpad_left) && dleft2){
            dleft2 = false;
            //usedistance = false;
            Drive(xcordset,ycordset,wcordset,true,false,0,0);

        }
        if(gamepad2.right_trigger > 0.6){
            slidecalibrated = false;
        }
        if(!gamepad2.b) Bbutton = true;
        if(gamepad2.b && Bbutton){
            Bbutton = false;
            math(xcordset,ycordset,wcordset,true);
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
        //telemetry.addData("servo", S0.getPosition());

        telemetry.update();

    }
    public void Drive(int xf, int yf, int wf, boolean savepos, boolean center, double xoffsetf, double yoffsetf) {
        double staggerf = 0;
        if (atwall || !auto) {
            staggerf = 0;
        } else {
            staggerf = stagger;
        }


        if (center && atwall){
            if (w == 1||w ==4){
                centerpos = centerposblue;
            }else{
                centerpos = centerposred;
            }
            Center(wf);
        }

        if (atwall) {
            target = 850;
        }
        else {
            drop();
            yoffsetf = 0;
            xoffsetf = 0;
        }
        math(xf, yf, wf,savepos);
        drive.setPoseEstimate(currentpose);
        if (!auto || D5.getState()||atwall) {

            traj = drive.trajectorySequenceBuilder(currentpose)
                    .back(b1)
                    .addDisplacementMarker(() -> {
                        if (atwall) {
                            slidecalibrated = false;
                            target = starget;
                        }
                        if (!atwall) {
                            target = hdata[xf + 5 * (yf - 1) + 2];
                            if (hdata[xf + 5 * (yf - 1) + 2] > 1000) {
                                S1.setPosition(UmbrellaMax1); //.7
                                S2.setPosition(UmbrellaMin2); //.03
                                ;//.7
                            }
                            if (target < 150) {  //if at ground station than drop cone and set slide up
                                S0.setPosition(camBothClosed);

                            }
                        }
                    })
                    .splineToSplineHeading(new Pose2d(x2, y2 - staggerf, o2), o2)
                    .addDisplacementMarker(() -> {
                        if (target < 150 && atwall == false) {  //if at ground station than drop cone and set slide up
                            target = 400;

                        }
                    })
                    .splineToSplineHeading(new Pose2d(x3 - .01 + xoffsetf, y3 - staggerf + yoffsetf, o3 + .01), o3)
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
            if (!atwall) {
                preset += 1;
                if (preset <= xcord.length && useiteration) {
                    xcordset = xm * xcord[preset - 1];
                    ycordset = ycord[preset - 1];
                }
                if (target > 500 && beacon) {
                    S0.setPosition(camTopOpen);
                } else {
                    S0.setPosition(camBothClosed);
                }

            }
            M0.setPower(0);
            M1.setPower(0);
            M2.setPower(0);
            M3.setPower(0);
            drive.waitForIdle();





            prevpose = drive.getPoseEstimate();
        }else{

            broke = true;
        }

    }
    public void Center(int wf){
        if (!D5.getState() && !broke) {
            servoclampasync = true;
        }
        drive.setPoseEstimate(new Pose2d());

        double distanceholder = 0;
        int count = 0;
        if (auto){
            target = 800;
            traj = drive.trajectorySequenceBuilder(new Pose2d())
                    .forward(5)
                    .build();
            drive.followTrajectorySequenceAsync(traj);
            drive.update();
            while(drive.isBusy()
                    && !isStopRequested()){
                drivestack();
                Slide();
            }
        }

        if (wf == 1 || wf == 3){

            traj = drive.trajectorySequenceBuilder(new Pose2d())
                    .back(.5)
                    .strafeLeft(100)
                    .build();

            drive.followTrajectorySequenceAsync(traj);
            drive.update();
            while(D4.getDistance(DistanceUnit.INCH) > 300
                    && !isStopRequested()){
                drivestack();
                ServoClampAsync();
            }
            /////////////////
            M1.setPower(0);
            M0.setPower(0);
            M2.setPower(0);
            M3.setPower(0);
            for(int i = 0;i < 20; i++) {
                distanceholder = D4.getDistance(DistanceUnit.INCH);
                if (distanceholder < 55 && distanceholder > 35) {
                    distance += distanceholder;
                    count += 1;
                }
            }

            distance = distance/count;
            //centered = true;
            //if (!auto) {
            telemetry.addData("centerdistance", distance);
            telemetry.update();
            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));
            traj = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                    //.lineToLinearHeading(new Pose2d(centerpos-distance,.5,Math.toRadians(90)))
                    .strafeRight(centerpos - distance)
                    .forward(4)
                    .build();
            /*}else{
                drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));
                traj = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                        //.lineToLinearHeading(new Pose2d(centerpos-distance,.5,Math.toRadians(90)))
                        .forward(4)
                        .build();
            }*/
                drive.followTrajectorySequenceAsync(traj);
                drive.update();
                while ((drive.isBusy() || servoclampasync)
                        && !isStopRequested()) {
                    drivestack();
                    ServoClampAsync();
                }


        }else{

            traj = drive.trajectorySequenceBuilder(new Pose2d())
                    .back(.5)
                    .strafeRight(100)
                    .build();

            drive.followTrajectorySequenceAsync(traj);
            drive.update();
            while(D2.getDistance(DistanceUnit.INCH) > 300
                    && !isStopRequested()){
                drivestack();
                ServoClampAsync();
            }
            /////////////////
            M1.setPower(0);
            M0.setPower(0);
            M2.setPower(0);
            M3.setPower(0);
            distance = 0;
            for(int i = 0;i < 20; i++) {
                distanceholder = D2.getDistance(DistanceUnit.INCH);
                if (distanceholder < 55 && distanceholder > 35) {
                    distance += distanceholder;
                    count += 1;
                }
            }
            distance = distance/count;
            //centered = true;
            telemetry.addData("centerdistance", distance);
            telemetry.update();
            //if (!auto) {
                drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));
                traj = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                        //.lineToLinearHeading(new Pose2d(-(centerpos-distance),.5,Math.toRadians(90)))
                        .strafeLeft(centerpos - distance)
                        .forward(4)
                        .build();

            /*}else{
                    drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));
                    traj = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                            //.lineToLinearHeading(new Pose2d(centerpos-distance,.5,Math.toRadians(90)))
                            .forward(4)
                            .build();
            }*/
                drive.followTrajectorySequenceAsync(traj);
                drive.update();
                while ((drive.isBusy() || servoclampasync)
                        && !isStopRequested()) {
                    drivestack();
                    ServoClampAsync();
                }

            }


    }
    public void math(int xf,int yf, int wf,boolean savepos) {

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
                b1 = x1 - ix;
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
                b1 = y1 - iy;
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
                b1 = y1 - iy;
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
                b1 = ix - x1;
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
            /*if (centered && auto){
                currentpose = new Pose2d(ix, iy-(centerpos-distance), io);
            }else {*/
            if (savepos && auto && !broke && drive.getPoseEstimate().getY() > -24 && drive.getPoseEstimate().getY() < 0){
                currentpose = new Pose2d(ix, drive.getPoseEstimate().getY(), io);
            }else{
                currentpose = new Pose2d(ix, iy, io);
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
                x2 = vx;// - offset;

            }
            if (w == 2) {
                ix = -12;
                iy = -64;
                io = Math.toRadians(-90);
                starget = 500;
                y2 = vy;// - offset;
                x2 = ix;

            }
            if (w == 3) {
                ix = 12;
                iy = -64;
                io = Math.toRadians(-90);
                starget = 500;
                y2 = vy;// - offset;
                x2 = ix;

            }
            if (w == 4) {
                ix = 64;
                iy = -12;
                io = 0;
                starget = 850;
                y2 = iy;
                x2 = vx;// + offset;


            }
            x1 = vx + reverseoffset * Math.cos(vo);
            y1 = vy + reverseoffset * Math.sin(vo);
            o1 = vo;
            o2 = io;
            x3 = ix;
            y3 = iy;
            o3 = io;
            b1 = d - reverseoffset;
            if (w==1) {
                x3 = x3 - dslam;
            }if (w == 2 || w == 3){
                y3 = y3 - dslam;
            }
            if ( w ==4){
                x3 = x3 + dslam;
            }



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
            if (savepos) {
                currentpose = prevpose;
            }else{
                currentpose = new Pose2d(vx + d * Math.cos(vo), vy + d * Math.sin(vo), vo);
            }
            if (!beacon){
                atwall = true;
            }
        }
    }
}