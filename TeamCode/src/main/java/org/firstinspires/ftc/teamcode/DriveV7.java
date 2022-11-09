package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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
@TeleOp(name = "DriveV7", group="Linear Opmode")

public class DriveV7 extends LinearOpMode {
    DcMotor M0;
    FtcDashboard dashboard;
    DcMotor M1;
    DcMotor M2;
    DcMotor M3;
    DcMotor M0_2;
    Servo S0;

    //slide sensor
    DigitalChannel D0;
    DistanceSensor D1; // front
    DistanceSensor D2; // right
    DistanceSensor D3; // back
    DistanceSensor D4; // left

    //public statics
    public static double d2 = 17; //distance strafe at pole but get interupted
    public static double d3 = 7; //distance come back off of pole
    public static double d4 = 1; //y offset when coming back
    public static double yoffset = 4;
    public static double Sset = 200; //test drop distance
    public static double dxoffset = 2.5;
    public static double dyoffset = -.5;

    //constants
    double d; // distance to pole update from sensor
    double target; //slide target position
    boolean track = false; //update d yes or no
    int y;   // y finallized when built
    int x;   // x finallized when built
    double xi = -.5;  //initial robot position against wall in coordinate system, either .5 or -.5
    double vy;  //vector roadrunner x value
    double vx;  //vector roadrunner y value
    double vo;  //target roadrunner theta
    boolean atwall = true; //used to know whether to run to or from
    int ycord = 2; // live cord y
    int xcord = 0; // live cord x
    boolean dup = false;
    boolean ddown = false;
    boolean dright = false;
    boolean dleft = false;
    boolean dslide = false;
    int[] hdata = new int[]{200, 1100, 200, 1100, 200,
                            1100, 1750, 2350, 1750, 1100,
                            200, 2350, 200, 2350, 200,
                            1100, 1750, 2350, 1750, 1100,
                            200, 1100, 200, 1100, 200}; //slide heights
    Trajectory t1;
    Trajectory t2;
    Trajectory t3;
    Trajectory t4;
    Trajectory f1;
    TrajectorySequence f15;
    Trajectory f2;
    Trajectory f3;
    Pose2d pole;

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

        //tension slide
        while (D0.getState() == true) {
            M0_2.setPower(.3);
        }
        while (D0.getState() == false) {
            M0_2.setPower(-0.2);
        }
        M0_2.setPower(0);
        M0_2.setDirection(DcMotor.Direction.FORWARD);
        M0_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M0_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M0_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //open clamp
        S0.setPosition(0.0);

        //waits here
        waitForStart();

        //raise slide on start
        target = 200;

        //main loop
        while (opModeIsActive()) {
            Drive();
            ServoClamp();
            Slide();
            Coordinates();
        }
    }

    public void ServoClamp() {
        //open cam
        if (gamepad1.left_bumper) {
            S0.setPosition(0.05);
        }

        //automatic clamping if within distance
        if ((target == 200) && (D1.getDistance(DistanceUnit.MM) <= 40) || gamepad1.right_bumper) {
            target = 0;
            while (Math.abs(target - M0_2.getCurrentPosition()) > 10) {
                M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250)))));
            }
            M0_2.setPower(0);
            S0.setPosition(0.25);
        }
    }

    public void Slide() {

        //slide update equation
        M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250)))));

        //set position with button
        if (gamepad1.a) {
            target = 200;
            S0.setPosition(0.05);
        }
        if (gamepad1.b) target = 2350;
        if (gamepad1.y) target = 1750;
        if (gamepad1.x) target = 1100;

        //slide reset if at switch
        if (D0.getState() && (target == 0)) {
            M0_2.setDirection(DcMotor.Direction.FORWARD);
            M0_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            M0_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            M0_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        //test drop onto pole
        if (gamepad1.dpad_down) dslide = true;
        if ((!gamepad1.dpad_down) && dslide) {
            dslide = false;
            target = target - Sset;
        }
    }


    public void Drive() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //standard drive
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

        //Automated Cordiante system
        if (gamepad1.right_stick_button) {

            //going to pole movement
            if (atwall) {
                track = false;
                //set varibles based off of cordinates
                y = ycord;
                x = xcord;
                vy = yoffset + 24 * (y - 1);
                //d2 = Math.abs(d2);
                if (x > 0) {
                    vx =  24 * Math.floor(Math.abs(x - xi));
                } else {
                    vx =  -24 * Math.floor(Math.abs(x - xi));
                }
                if (x > xi) {
                    vo = 90;
                } else {
                    vo = -90;
                }


                //build non x translation
                if(vx == 0){
                    t1 = drive.trajectoryBuilder(new Pose2d(0,0,0))
                            .back(vy + d2)
                            .addDisplacementMarker(Math.abs(vy),() ->{
                                track = true;
                            })
                            .build();
                }

                //build with x translation
                else{
                    d2 = Math.abs(d2);
                    vo = 180;
                    if (x < xi){
                        d2 = -d2;
                    }
                    t1 = drive.trajectoryBuilder(new Pose2d(0,0,0))
                            .back(vy)
                            .addDisplacementMarker(() -> {
                                drive.followTrajectoryAsync(t2);
                            })
                            .build();
                    t2 = drive.trajectoryBuilder(t1.end())
                            .strafeLeft(vx+d2)
                            .addDisplacementMarker(Math.abs(vx),() ->{
                                track = true;
                            })
                            .build();
                }

                //close clamp
                S0.setPosition(.3);

                //retrieve height of pole
                target = hdata[x + 5*(y-1)+2];

                //movement 1 with distance interrupt
                drive.followTrajectoryAsync(t1);
                drive.update();
                while (Math.abs(gamepad1.left_stick_x) < .5
                        && Math.abs(gamepad1.left_stick_y) < .5
                        && Math.abs(gamepad1.right_stick_x) < .5
                        && Math.abs(gamepad1.right_stick_y) < .5
                        && drive.isBusy()
                ) {
                    if(D4.getDistance(DistanceUnit.INCH) <= 10 && D4.getDistance(DistanceUnit.INCH) >=1 && track && vo == 90){
                        d = D4.getDistance(DistanceUnit.INCH);
                        break;
                    }
                    if(D2.getDistance(DistanceUnit.INCH) <= 10 && D2.getDistance(DistanceUnit.INCH) >=1 && track && vo == -90){
                        d = D2.getDistance(DistanceUnit.INCH);
                        break;
                    }
                    if(D3.getDistance(DistanceUnit.INCH) <= 10 && D3.getDistance(DistanceUnit.INCH) >=1 && track && vo == 180){
                        d = D3.getDistance(DistanceUnit.INCH);
                        break;
                    }
                    drive.update();
                    Slide();
                    Coordinates();
                    while (gamepad1.right_stick_button){
                        M0.setPower(0);
                        M3.setPower(0);
                        M1.setPower(0);
                        M2.setPower(0);
                    }
                }

                //movement 2 build
                drive.setPoseEstimate(new Pose2d());
                if(d >= 10 || d <= 1){
                    d = 5;
                }
                t4 = drive.trajectoryBuilder(new Pose2d(dyoffset,(dxoffset)*Math.sin(Math.toRadians(vo)),Math.toRadians(vo)))
                        .lineToLinearHeading(new Pose2d(dyoffset,(d+dxoffset)*Math.sin(Math.toRadians(vo)),Math.toRadians(vo)))
                        .addDisplacementMarker(() ->{
                        })
                        .build();

                //movement 2 onto pole
                drive.followTrajectoryAsync(t4);
                drive.update();
                while (Math.abs(gamepad1.left_stick_x) < .5
                        && Math.abs(gamepad1.left_stick_y) < .5
                        && Math.abs(gamepad1.right_stick_x) < .5
                        && Math.abs(gamepad1.right_stick_y) < .5
                        && drive.isBusy()
                ) {
                    drive.update();
                    Slide();
                    Coordinates();
                    while (gamepad1.right_stick_button){
                        M0.setPower(0);
                        M3.setPower(0);
                        M1.setPower(0);
                        M2.setPower(0);
                    }
                }
                target = target - 250;
                S0.setPosition(0);
                atwall = false;
            }

            //movement returning to wall
            else {
                //set pose at
                pole = new Pose2d(-vy - 12,vx + Math.sin(Math.toRadians(vo))*d3 ,Math.toRadians(vo));
                drive.setPoseEstimate(pole);
                f1 = drive.trajectoryBuilder(pole)
                        .back(d3)
                        .addDisplacementMarker(() -> {
                            target = 200;
                            drive.followTrajectorySequence(f15);
                        })
                        .build();
                f15 = drive.trajectorySequenceBuilder(f1.end())
                        .turn(Math.toRadians(-vo))
                        .addDisplacementMarker(() -> {
                              drive.followTrajectoryAsync(f2);
                        })
                        .build();

                //neeeeeeds to be fixed but build back
                if(vx == 0){
                    f2 = drive.trajectoryBuilder(f15.end())
                            .addDisplacementMarker(() -> {
                                target = 200;
                            })
                            .lineToLinearHeading(new Pose2d(d4,0,0))
                            .build();
                }
                else{
                    f2 = drive.trajectoryBuilder(f15.end())
                            .lineToLinearHeading(new Pose2d(-vy, 0, 0))
                            .addDisplacementMarker(() -> {
                                target = 200;
                                drive.followTrajectoryAsync(f3);
                            })
                            .build();
                    //move diagonal forwards to target junction
                    f3 = drive.trajectoryBuilder(f2.end())
                            .lineToLinearHeading(new Pose2d(d4,0,0))
                            .build();

                }

                //open cam
                S0.setPosition(0);

                //return movement
                drive.followTrajectoryAsync(f1);
                drive.update();
                while (Math.abs(gamepad1.left_stick_x) < .5
                        && Math.abs(gamepad1.left_stick_y) < .5
                        && Math.abs(gamepad1.right_stick_x) < .5
                        && Math.abs(gamepad1.right_stick_y) < .5
                        && drive.isBusy())
                {
                    drive.update();
                    Slide();
                    Coordinates();
                    while (gamepad1.right_stick_button){
                        M0.setPower(0);
                        M3.setPower(0);
                        M1.setPower(0);
                        M2.setPower(0);
                    }
                }
                atwall = true;
            }

        }

    }

    public void Coordinates () {

        //toggle cords
        if (gamepad2.dpad_up) dup = true;
        if (gamepad2.dpad_down) ddown = true;
        if (gamepad2.dpad_left) dleft = true;
        if (gamepad2.dpad_right) dright = true;

        //field inital position
        if (gamepad2.right_bumper) {
            xi = .5;
        }
        if (gamepad2.left_bumper) {
            xi = -.5;
        }

        //set game field cordinates
        if ((!gamepad2.dpad_up) && dup) {
            dup = false;
            ycord += 1;
        }
        if ((!gamepad2.dpad_down) && ddown) {
            ddown = false;
            ycord -= 1;
        }
        if ((!gamepad2.dpad_right) && dright) {
            dright = false;
            xcord += 1;
        }
        if ((!gamepad2.dpad_left) && dleft) {
            dleft = false;
            xcord -= 1;
        }

        //all telemetry
        telemetry.addData("x", xcord);
        telemetry.addData("y", ycord);
        telemetry.addData("xi", xi);
        telemetry.addData("atwall", atwall);
        telemetry.addData("front", D1.getDistance(DistanceUnit.INCH));
        telemetry.addData("right", D2.getDistance(DistanceUnit.INCH));
        telemetry.addData("left", D4.getDistance(DistanceUnit.INCH));
        telemetry.addData("d",d);
        telemetry.addData("track",track);
        telemetry.addData("vy",vy);
        telemetry.update();

    }
}

