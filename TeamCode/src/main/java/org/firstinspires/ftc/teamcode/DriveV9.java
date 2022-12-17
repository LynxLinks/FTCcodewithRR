package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.google.gson.annotations.Until;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.teamcode.Auto13.sidered;
import static org.firstinspires.ftc.teamcode.Auto13.vopark;
import static org.firstinspires.ftc.teamcode.Auto13.slidespeed;
import static org.firstinspires.ftc.teamcode.Auto13.autopose;
import static org.firstinspires.ftc.teamcode.TestServos.UmbrellaMax1;
import static org.firstinspires.ftc.teamcode.TestServos.UmbrellaMin1;
import static org.firstinspires.ftc.teamcode.TestServos.UmbrellaMax2;
import static org.firstinspires.ftc.teamcode.TestServos.UmbrellaMin2;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Disabled
@Config
@TeleOp(name = "DriveV9", group="Linear Opmode")
public class DriveV9 extends LinearOpMode {
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

    TrajectorySequence traj;
    TrajectorySequence setuptraj;
    Pose2d currentpose;

    public static double d1 = 11.5;
    public static double d2 = .2;
    public static double Sdrop = 400;
    public static double offset = 9;
    public static double reverseoffset = 8;
    public static boolean usepreset = false;
    public static double bump = 250;
    public static double slideoffset = 950;

    int[] xcord = new int[]{-1,0,-1,0,1,0};
    int [] ycord = new int[]{3,2,1,1,2,1,2};

    int preset = 1;
    boolean atwall = true;
    double starget = 850;
    int[] hdata = {100, 1150, 100, 1150, 100,
            1150, 1750, 2250, 1750, 1150,
            100, 2250, 100, 2250, 100,
            1150, 1750, 2250, 1750, 1150,
            100, 1150, 100, 1150, 100
            ,200,200,200,200,200,200,200,200,200,200,200};

    int x;
    int y;
    int w;
    boolean yfirst;
    double target;
    double vx;
    double vo;
    double vy;
    double d;
    double ix;
    double iy;
    double io;
    double x1;
    double x2;
    double x3;
    double x4;
    double y1;
    double y2;
    double y3;
    double y4;
    double o1;
    double o2;
    double o3;
    double o4;
    boolean beacon;
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
    boolean gx;
    boolean gy;
    int xm;
    int wcordset;
    int ycordset;
    int xcordset;

    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
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
        D5 = hardwareMap.get(DigitalChannel.class, "D5");
        D1 = hardwareMap.get(DistanceSensor.class, "D1");
        D2 = hardwareMap.get(DistanceSensor.class, "D2");
        D4 = hardwareMap.get(DistanceSensor.class, "D4");
        D3 = hardwareMap.get(DistanceSensor.class, "D3");
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

        S0.setPosition(0.0);
        S1.setPosition(0.02);
        S2.setPosition(.7);


        if (sidered){
            wcordset = 1;
            xm = 1;
        } else {
            wcordset = 4;
            xm = -1;
        }
        ycordset = ycord[0];
        xcordset = xm*xcord[0];

        if (usepreset){  //if using automatic move to stack
            drive.setPoseEstimate(autopose);
            setuptraj = drive.trajectorySequenceBuilder(autopose)
                    .lineToLinearHeading(new Pose2d(-65, -12,vopark))
                    .build();
            drive.followTrajectorySequenceAsync(setuptraj);
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
                UI();
            }
        }

        math();
        math();

        waitForStart();


        while (opModeIsActive()) {
            Slide();
            UI();
            manual();
            ServoTrigger();
        }
    }
    public void ServoClamp() {
        S0.setPosition(0.21);
        M0_2.setPower(-.5);
        while (D5.getState() == false && M0_2.getCurrentPosition() > -150){
        }
        if (w == 1 || w ==4){
            S0.setPosition(.37);
        }
        else {
            S0.setPosition(.47);
        }
        //telemetry.addData("current",M0_2.getCurrentPosition());
        target = M0_2.getCurrentPosition() - bump;
        //telemetry.addData("target",target);
        //telemetry.update();
        UntilSlide();
        target = target + slideoffset;
        UntilSlide();



    }
    public void Slide () {

        if (slidecalibrated) {
            M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - 1.4*M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - 1.4*M0_2.getCurrentPosition()) / 250)))));
        } else {
            if (D0.getState() == true && !beenoff) {
                M0_2.setPower(.4);
            }
            if (D0.getState() == false) {
                M0_2.setPower(-0.4);
                beenoff = true;
            }
            if (D0.getState() == true && beenoff) {
                M0_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                M0_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                M0_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                slidecalibrated = true;
                beenoff = false;
                M0_2.setPower(0);
            }
        }
    }
    public void Drive() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        if(atwall){
            target = 850;
        }
        else{
            drop();
        }

        math();

        drive.setPoseEstimate(currentpose);

        traj = drive.trajectorySequenceBuilder(currentpose)
                .lineToLinearHeading(new Pose2d(x1 + 0.02,y1 + 0.02,o1))
                .addDisplacementMarker(() ->{
                    if (atwall){
                        slidecalibrated = false;
                        target = starget;
                    }
                })
                .lineToLinearHeading(new Pose2d(x2 + .01,y2 + .01,o2))
                .lineToLinearHeading(new Pose2d(x3 + .01,y3 + .03,o3))
                .addDisplacementMarker(() ->{
                    if (!atwall){
                        target = hdata[x + 5*(y-1)+2];
                        if(hdata[x + 5*(y-1)+2] > 1000){
                            S1.setPosition(UmbrellaMax1); //.02
                            S2.setPosition(UmbrellaMin2); ;//.7
                        }
                    }
                })
                .lineToLinearHeading(new Pose2d(x4,y4,o4))
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
                && Math.abs(gamepad1.left_trigger) < .5)

        {
            drive.update();
            Slide();
            UI();
        }
        if (!atwall && target > 500 && !beacon){
            S0.setPosition(0);
        }
    }
    public void manual(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //manual drive
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y -0.2*gamepad1.right_stick_y,
                        -gamepad1.left_stick_x-0.2*gamepad1.right_stick_x,
                        0.5*gamepad1.left_trigger - 0.5*gamepad1.right_trigger
                )
        );

        drive.update();
    }
    public void UntilSlide() {
        if ((target - 1.4*M0_2.getCurrentPosition()) > 0) {
            M0_2.setPower(slidespeed);
            while (target > 1.4*M0_2.getCurrentPosition()){
                UI();
                manual();
            };
        }
        else{
            M0_2.setPower(-slidespeed );
            while (target < 1.4*M0_2.getCurrentPosition()){
                UI();
                manual();
            }
        }
        M0_2.setPower(0);
    }
    public void drop(){

        double pt = target;
        target = target - Sdrop;
        UntilSlide();
        if (beacon){
            //only drop b e con
            S0.setPosition(.37);
            beacon = false;
        }
        else {
            S0.setPosition(0.18);
        }
        S1.setPosition(UmbrellaMin1); //.02
        S2.setPosition(UmbrellaMax2); ;//.7
        target = pt;
        UntilSlide();
    }
    public void ServoTrigger() {
        if (gamepad1.right_bumper ) {
            if (atwall) {
                ServoClamp();
            } else {
                drop();
            }
        }
        if (D5.getState() == false && D1.getDistance(DistanceUnit.INCH) < 1.75 && atwall){
            ServoClamp();
        }
    }

    public void UI() {
        StandardTrackingWheelLocalizer myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap);


        //Manual Servo
        if (gamepad1.left_bumper) {
            S0.setPosition(0.18);
        }

        //Manual Slide
        if (gamepad1.a) target = starget;
        if (gamepad1.b) target = 2250;
        if (gamepad1.y) target = 1750;
        if (gamepad1.x) target = 1350;

        //Mauanl Umbrella
        if (gamepad2.left_stick_button){
            S1.setPosition(UmbrellaMax1); //.7
            S2.setPosition(UmbrellaMin2); //.03
        }if (gamepad2.right_stick_button){
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
        if(gamepad2.left_trigger > 0.6){
            atwall = true;
        }
        if((gamepad1.dpad_right) && dright2){
            dright2 = false;
            yfirst = true;
            Drive();
        }
        if(gamepad2.right_trigger > 0.6){
            slidecalibrated = false;
        }
        if(!gamepad2.b) Bbutton = true;
        if(gamepad2.b && Bbutton){
            Bbutton = false;
            math();
        }
        myLocalizer.update();

        Pose2d myPose = myLocalizer.getPoseEstimate();

        telemetry.addData("x", xcordset);
        telemetry.addData("", "");
        telemetry.addData("y", ycordset);
        telemetry.addData("", "");
        telemetry.addData("w", wcordset);
        telemetry.addData("", "");
        telemetry.addData("atwall", atwall);
        telemetry.addData("", "");
        telemetry.addData("beacon", beacon);
       // telemetry.addData("x", myPose.getX());
       // telemetry.addData("y", myPose.getY());
        //telemetry.addData("front", D1.getDistance(DistanceUnit.INCH));
        //telemetry.addData("Target", target);
        telemetry.update();

    }
    //z
    public void math(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        y = ycordset;
        x = xcordset;
        w = wcordset;
        if (atwall) {
            if (y ==6){
                d = d2;
            }
            else{
                d = d1;
            }
            if(w == 1){
                yfirst=false;
                if (y >= 3){


                    vy = 24 * (y - 3) -12;
                    if (x <= -2){
                        vx = 24 * (x+1) - 12;
                        vo = Math.toRadians(135);
                    }
                    else{


                        vx = 24 * x - 12;
                        vo = Math.toRadians(45);
                    }
                }
                else{
                    vy = 24 * (y - 2) -12;
                    if (x <= -2){
                        vx = 24 * (x+1) - 12;
                        vo = Math.toRadians(-135);
                    }
                    else{
                        vx = 24 * x - 12;
                        vo = Math.toRadians(-45);
                    }
                }
            }
            if(w == 2){

                if (x >= 0){

                    vx = 24 * x -12;
                    if (y <= 1){
                        vy = 24 * (y -2) - 12;
                        vo = Math.toRadians(-45);
                    }else{
                        vy = 24 * (y -3) - 12;
                        vo = Math.toRadians(45);
                    }
                }
                else{

                    vx = 24 * (x + 1) -12;
                    if (y <= 1){
                        vy = 24 * (y -2) - 12;
                        vo = Math.toRadians(-135);
                    }else{
                        vy = 24 * (y -3) - 12;
                        vo = Math.toRadians(135);
                    }
                }
            }
            if(w == 3) {
                if (x >= 1) {
                    vx = 24 * (x - 1) + 12;
                    if (y <= 1){
                        vy = 24 * (y -2) - 12;
                        vo = Math.toRadians(-45);
                    }else{
                        vy = 24 * (y -3) - 12;
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

            }
            if(w == 4){
                yfirst = false;
                if (y >= 3){
                    vy = 24 * (y - 3) - 12;
                    if (x >= 2){
                        vx = 24 * (x-1) + 12;
                        vo = Math.toRadians(45);
                    }
                    else{
                        vx = 24 * x + 12;
                        vo = Math.toRadians(135);
                    }
                }
                else{
                    vy = 24 * (y - 2) -12;
                    if (x >= 2){
                        vx = 24 * (x-1) + 12;
                        vo = Math.toRadians(-45);
                    }
                    else{
                        vx = 24 * x + 12;
                        vo = Math.toRadians(-135);
                    }


                }
            }
            if (y == 6){
                vx = vx -2*xm;
            }
            if(yfirst){
                x1 = ix;
                y1 = vy;
            }
            else{
                x1 = vx;
                y1 = iy;
            }

            x2 = vx;
            y2 = vy;
            x3 = vx;
            y3 = vy;
            x4 = vx + d * Math.cos(vo);
            y4 = vy + d * Math.sin(vo) ;
            o1 = io;
            o2 = io;
            o3 = vo;
            o4 = vo;
            currentpose = new Pose2d(ix,iy,io);
            atwall = false;
        }
        else {
            if(w == 1){
                ix = -65;
                iy = -12;
                io = Math.toRadians(180);
                starget = 850;
                yfirst = true;

            }
            if(w == 2){
                ix = -12;
                iy = -65;
                io = Math.toRadians(-90);
                starget = 500;
                yfirst = false;
            }
            if(w == 3){
                ix = 12;
                iy = -65;
                io = Math.toRadians(-90);
                starget = 500;
                yfirst = false;

            }
            if(w == 4){
                ix = 65;
                iy = -12;
                io = 0;
                starget = 850;
                yfirst = true;
            }


            if(yfirst){
                x3 = vx;
                y3 = iy;

            } else{
                x3 = ix;
                y3 = vy;
            }

            x1 = vx;
            y1 = vy;
            x2 = vx;
            y2 = vy;
            x4 = ix;
            y4 = iy;
            o1 = vo;
            o2 = io;
            o3 = io;
            o4 = io;

            currentpose = new Pose2d(vx + d*Math.cos(vo),vy + d*Math.sin(vo),vo);
            atwall = true;
        }
    }
}


