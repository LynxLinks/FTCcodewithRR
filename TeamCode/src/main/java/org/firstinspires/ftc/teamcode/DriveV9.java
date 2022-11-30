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
import static org.firstinspires.ftc.teamcode.Auto12.sidered;
import static org.firstinspires.ftc.teamcode.Auto12.vopark;
import static org.firstinspires.ftc.teamcode.Auto12.slidespeed;
import static org.firstinspires.ftc.teamcode.Auto12.autopose;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Config
@TeleOp(name = "DriveV8", group="Linear Opmode")
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

    public static double d1 = 11.5;
    public static double d2 = .2;
    public static double Sdrop = 350;
    public static double offset = 9;
    public static double reverseoffset = 8;
    public static double slideoffset = 300;
    public static boolean usepreset = false;

    int[] xcord = new int[]{-1,0,-1,0,1,0};
    int [] ycord = new int[]{3,2,1,1,2,1,2};

    int x;
    int y;
    int w;
    int preset = 1;
    boolean atwall = true;
    double target;
    double starget = 850;
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
    double y1;
    double y2;
    double y3;
    double o1;
    double o2;
    double o3;
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

    TrajectorySequence traj;
    TrajectorySequence setuptraj;
    Pose2d currentpose;
    int[] hdata = {100, 1300, 100, 1300, 100,
            1300, 1950, 2550, 1950, 1300,
            100, 2550, 100, 2550, 100,
            1300, 1950, 2550, 1950, 1300,
            100, 1300, 100, 1300, 100
            ,200,200,200,200,200,200,200,200,200,200,200};
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
        }
    }
    public void ServoClamp() {

            M0_2.setPower(-1);
            while (D5.getState() == false);
            if (w == 4 || w ==1){
                //close upper
            }
            else{
                //close both
            }
            M0_2.setPower(1);
            target = M0_2.getCurrentPosition() + slideoffset;
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

        if(atwall)target = 850;
        else drop();

        math();
        traj = drive.trajectorySequenceBuilder(currentpose)
                .lineToLinearHeading(new Pose2d(x1, y1,o1))
                .addDisplacementMarker(() ->{
                    if (atwall){
                        slidecalibrated = false;
                        target = starget;
                    }
                    if (!atwall){


                    target = hdata[x + 5*(y-1)+2];
                    if(hdata[x + 5*(y-1)+2] > 1000){
                        S1.setPosition(.7); //.02
                        S2.setPosition(.03); ;//.7
                    }
                    }
                })
                .splineToSplineHeading(new Pose2d(x2, y2, o2), o2)
                .addDisplacementMarker(() ->{
                    if (target < 150 && atwall == false){  //if at ground station than drop cone and set slide up
                        S0.setPosition(0);
                        target = 800;
                    }
                })
                .splineToSplineHeading(new Pose2d(x3-.01, y3-.01, o3 +.01), o3)
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
                drive.update();
                Slide();
                UI();
            }
            if (!atwall) {
                preset += 1;
                xcordset = xm * xcord[preset - 1];
                ycordset = ycord[preset - 1];
                if (target > 500 && !beacon) {
                    S0.setPosition(0);
                }

        }

    }
    public void manual(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //manual drive
        double yAxis;
        double xAxis;
        double Rotate;
        yAxis = gamepad1.left_stick_y * .8 + gamepad1.right_stick_y / 3;
        xAxis = gamepad1.left_stick_x * .8 + gamepad1.right_stick_x / 3;
        Rotate = -gamepad1.left_trigger / 2 + gamepad1.right_trigger / 2;
        if (!drive.isBusy()) {
            M0.setPower((Rotate + (-yAxis + xAxis)));
            M3.setPower((Rotate + (-yAxis - xAxis)));
            M1.setPower(-(Rotate + (yAxis + xAxis)));
            M2.setPower(-(Rotate + (yAxis - xAxis)));
        }
    }
    public void UntilSlide() {
        if ((target - 1.4*M0_2.getCurrentPosition()) > 0) {
            M0_2.setPower(slidespeed);
            while (target > 1.4*M0_2.getCurrentPosition()) ;
        }
        else{
            M0_2.setPower(-slidespeed );
            while (target < 1.4*M0_2.getCurrentPosition());
        }
        M0_2.setPower(0);
    }
    public void drop(){

        double pt = target;
        target = target - Sdrop;
        UntilSlide();
        if (beacon){
            //only drop beacon
            beacon = false;
        }
        else {
            S0.setPosition(0);
        }
        S1.setPosition(0.02); //.02
        S2.setPosition(.7); ;//.7
        target = pt;
        UntilSlide();
    }

    public void UI() {
        if (D5.getState() == false && D1.getDistance(DistanceUnit.MM) < 33 && target <= 1000){
            ServoClamp();
        }

        //Manual Servo
        if (gamepad1.left_bumper) {
            S0.setPosition(0.05);
        }
        if (gamepad1.right_bumper ) {
            if (atwall) {
                ServoClamp();
            } else {
                drop();
            }
        }

        //Manual Slide
        if (gamepad1.a) target = starget;
        if (gamepad1.b) target = 2550;
        if (gamepad1.y) target = 1950;
        if (gamepad1.x) target = 1300;

        //Mauanl Umbrella
        if (gamepad2.left_stick_button){
            S1.setPosition(.7); //.02
            S2.setPosition(.03); ;//.7
        }if (gamepad2.right_stick_button){
            S1.setPosition(0.02); //.02
            S2.setPosition(.7); ;//.7
        }


        //Cordianates
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
        if (gamepad2.right_bumper && w < 4 && dbright) {
            dbright = false;
            wcordset += 1;
        }
        if (gamepad2.left_bumper && w > 1 && dbleft) {
            dbleft = false;
            wcordset -= 1;
        }
        if ((gamepad2.dpad_up) && dup) {
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
        if(gamepad2.right_stick_y < .5){
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
        telemetry.addData("x", x);
        telemetry.addData("", "");
        telemetry.addData("y", y);
        telemetry.addData("", "");
        telemetry.addData("w", w);
        telemetry.addData("", "");
        telemetry.addData("atwall", atwall);
        telemetry.update();
    }
    public void math() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
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
                        vx = 24 * (x - 1) + 12;
                        vo = Math.toRadians(45);
                    } else {
                        vx = 24 * x + 12;
                        vo = Math.toRadians(135);
                    }
                } else {
                    vy = 24 * (y - 2) - 12;
                    if (x >= 2) {
                        vx = 24 * (x - 1) + 12;
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
            currentpose = new Pose2d(ix, iy, io);
            atwall = false;

        } else {
            if (w == 1) {
                ix = -65;
                iy = -12;
                io = Math.toRadians(180);
                starget = 850;
                y2 = iy;
                x2 = vx - offset;
            }
            if (w == 2) {
                ix = -12;
                iy = -65;
                io = Math.toRadians(-90);
                starget = 500;
                y2 = vy - offset;
                x2 = ix;
            }
            if (w == 3) {
                ix = 12;
                iy = -65;
                io = Math.toRadians(-90);
                starget = 500;
                y2 = vy - offset;
                x2 = ix;

            }
            if (w == 4) {
                ix = 65;
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
            currentpose = new Pose2d(vx + d * Math.cos(vo), vy + d * Math.sin(vo), drive.getRawExternalHeading());
            atwall = true;
        }
    }

}


