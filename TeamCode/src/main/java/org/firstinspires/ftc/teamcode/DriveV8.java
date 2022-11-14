package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

//name and class

@Config
@TeleOp(name = "DriveV8", group="Linear Opmode")

public class DriveV8 extends LinearOpMode {
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

    public static double d = 11.5;
    public static double Sdrop = 350;

    int y = 2;
    int x = 0;
    int w = 4;
    double vy; // sync with auto
    double vx; // sync with auto
    boolean atwall = false;
    double target;
    double vo;
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
    boolean yfirst;
    boolean dup;
    boolean ddown;
    boolean dright;
    boolean dleft;
    boolean dbright;
    boolean dbleft;
    boolean dslide;
    boolean slidecalibrated;
    boolean beenoff;
    TrajectorySequence traj;
    Pose2d currentpose;
    int[] hdata = new int[]{400, 1300, 400, 1300, 400,
            1300, 1950, 2550, 1950, 1300,
            400, 1300, 400, 1300, 400,
            1300, 1950, 2550, 1950, 1300,
            400, 1300, 400, 1300, 400};


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
        S1.setPosition(0);
        S2.setPosition(0.75);

        if(w == 1){
            ix = -65;
            iy = -12;
            io = Math.toRadians(180);
        }
        if(w == 2){
            ix = -12;
            iy = -65;
            io = Math.toRadians(-90);
        }
        if(w == 3){
            ix = 12;
            iy = -65;
            io = Math.toRadians(-90);
        }
        if(w == 4){
            ix = 65;
            iy = -12;
            io = 0;
        }
        waitForStart();

        while (opModeIsActive()) {
            ServoClamp();
            Slide();
            UI();
        }
    }

    public void ServoClamp() {
        if ((target <= 600) && (D1.getDistance(DistanceUnit.MM) <= 33)) {
            double prevtarget = target;
            target = 0;
            while (Math.abs(target - M0_2.getCurrentPosition()) > 10) {
                M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250)))));
            }
            M0_2.setPower(0);
            S0.setPosition(0.25);
            target = prevtarget;
        }
    }

    public void Slide () {

        if (slidecalibrated) {
            M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250)))));
        }

        else {
            if (D0.getState() == true && !beenoff) { //if slide is on limit swtich
                M0_2.setPower(.5);
            }
            if (D0.getState() == false) { //if slide is above limit
                M0_2.setPower(-0.5);
                beenoff = true;
            }
            if (D0.getState() == true && beenoff) { //if slide is on limit and calibratedM0_2.setDirection(DcMotor.Direction.FORWARD);
                M0_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                M0_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                M0_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                slidecalibrated = true;
                beenoff = false;
            }
        }
    }

    public void Drive() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        if (atwall) {
            if(w == 1){
                vx = 24 * x - 12;

                if (y >= 3){
                    vo = Math.toRadians(45);
                    vy = 24 * (y - 3) -12;
                }
                else{
                    vo = Math.toRadians(-45);
                    vy = 24 * (y - 2) -12;
                }
            }
            if(w == 2){
                vy = 24 * (y -3) - 12;
                if (x >= 0){
                    vo = Math.toRadians(45);
                    vx = 24 * x -12;
                }
                else{
                    vo = Math.toRadians(135);
                    vx = 24 * (x + 1) -12;
                }
            }
            if(w == 3){
                vy = 24 * (y -3) - 12;
                if (x >= 1){
                    vo = Math.toRadians(45);
                    vx = 24 * (x - 1) + 12;
                }
                else{
                    vo = Math.toRadians(135);
                    vx = 24 * x  + 12;
                }
            }
            if(w == 4){
                vx = 24 * x + 12;

                if (y >= 3){
                    vo = Math.toRadians(135);
                    vy = 24 * (y - 3) - 12;
                }
                else{
                    vo = Math.toRadians(-135);
                    vy = 24 * (y - 2) -12;
                }
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
            x3 = vx + d * Math.cos(vo);
            y3 = vy + d * Math.sin(vo) ;
            o1 = vo;
            o2 = vo;
            currentpose = new Pose2d(ix,iy,io);
            atwall = false;
            target = hdata[x + 5*(y-1)+2];
        }
        else {
            if(w == 1){
                ix = -65;
                iy = -12;
                io = Math.toRadians(180);
                target = 600;

            }
            if(w == 2){
                ix = -12;
                iy = -65;
                io = Math.toRadians(-90);
                target = 300;
            }
            if(w == 3){
                ix = 12;
                iy = -65;
                io = Math.toRadians(-90);
                target = 300;
            }
            if(w == 4){
                ix = 65;
                iy = -12;
                io = 0;
                target = 600;

            }


            if(yfirst){
                x2 = vx;
                y2 = iy;

            }
            else{
                x2 = ix;
                y2 = vy;
            }

            x1 = vx;
            y1 = vy;
            x3 = ix;
            y3 = iy;
            o1 = vo;
            o2 = io;

            currentpose = new Pose2d(vx + d*Math.cos(vo),vy + d*Math.sin(vo),vo);
            atwall = true;
            S0.setPosition(0); //drop and up on umbrella
            S1.setPosition(0);
            S2.setPosition(0.75);

        }

        drive.setPoseEstimate(currentpose);

        traj = drive.trajectorySequenceBuilder(currentpose)
                .lineToLinearHeading(new Pose2d(x1,y1,o1))
                .addDisplacementMarker(Math.abs(vx),() ->{
                    if (atwall){
                        slidecalibrated = false;
                    }
                })
                .lineToLinearHeading(new Pose2d(x2,y2,o1))
                .addDisplacementMarker(Math.abs(vx),() ->{
                    if (!atwall){
                        S1.setPosition(0.75);
                        S2.setPosition(0);
                    }
                })
                .lineToLinearHeading(new Pose2d(x3,y3,o1))
                .addDisplacementMarker(Math.abs(vx),() ->{
                    if (!atwall){
                        target = target-Sdrop;
                    }
                })
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
                && Math.abs(gamepad1.left_trigger) < .5
        )

        {
            drive.update();
            Slide();
            //UI();
        }

    }
    public void UI() {
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
        //Manual Servo
        if (gamepad1.left_bumper) {
            S0.setPosition(0.05);
        }
        if (gamepad1.right_bumper) {
            target = 0;
            while (Math.abs(target - M0_2.getCurrentPosition()) > 10) {
                M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250)))));
            }
            M0_2.setPower(0);
            S0.setPosition(0.25);
        }

        //Manual Slide
        if (gamepad1.a) target = 200;
        if (gamepad1.b) target = 2550;
        if (gamepad1.y) target = 1950;
        if (gamepad1.x) target = 1300;

        //Mauanl Umbrella
        if (gamepad2.a){
            S1.setPosition(0.75);
            S2.setPosition(0);
        }if (gamepad2.b){
            //down
            //up
            S1.setPosition(0);

            S2.setPosition(0.75);
        }

        //Manual drop
        if ((!gamepad1.dpad_down) && dslide) {
            dslide = false;
            target = target - Sdrop;
        }
        if ((!gamepad1.dpad_down) && !dslide) {
            dslide = true;
            target = target + Sdrop;
        }

        //Cordianates
        if (gamepad2.dpad_up) dup = true;
        if (gamepad2.dpad_down) ddown = true;
        if (gamepad2.dpad_left) dleft = true;
        if (gamepad2.dpad_right) dright = true;
        if (gamepad2.right_bumper) dbright = true;
        if (gamepad2.left_bumper) dbleft = true;
        if (!gamepad2.right_bumper && w < 4 && dbright) {
            dbright = false;
            w += 1;
        }
        if (!gamepad2.left_bumper && w > 1 && dbleft) {
            dbleft = false;
            w -= 1;
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
        if(gamepad2.a){
            x = 0;
            y = 2;
        }
        if(gamepad1.dpad_left){
            yfirst = false;
            Drive();
        }
        if(gamepad1.dpad_right){
            yfirst = true;
            Drive();
        }

        //Teletry
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("w", w);
        telemetry.addData("atwall", atwall);
        telemetry.addData("front", D1.getDistance(DistanceUnit.INCH));
        telemetry.addData("right", D2.getDistance(DistanceUnit.INCH));
        telemetry.addData("left", D4.getDistance(DistanceUnit.INCH));
        telemetry.update();

    }
}

