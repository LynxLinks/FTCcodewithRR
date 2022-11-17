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

//name and class

@Disabled
@Config
@TeleOp(name = "DriveV4", group="Linear Opmode")

public class DriveV4 extends LinearOpMode {
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
    public static double yoffset = 1;  //constant added to all y positions
    public static double d = 11.5;
    public static double Sset = 200;
    //diagonal distance forward and backward
    int y = 2;   //y coordinate input
    int x = 0;   //x coordinate input
    int ycord = 2;
    int xcord = 0;
    double target; //slide target position
    double vy = 1;  //vector roadrunner x value
    double vx = 1;  //vector roadrunner y value
    double vo = 1;  //target roadrunner theta
    double xi = -.5;  //initial robot position against wall in coordinate system, either .5 or -.5
    /*int[] hdata = new int[]{200, 1100, 200, 1100, 200,
            1100, 1750, 2350, 1750, 1100,
            200, 2350, 200, 2350, 200,
            1100, 1750, 2350, 1750, 1100,
            200, 1100, 200, 1100, 200};

     */
    int[] hdata = new int[]{400, 1300, 400, 1300, 400,
            1300, 1950, 2550, 1950, 1300,
            400, 1300, 400, 1300, 400,
            1300, 1950, 2550, 1950, 1300,
            400, 1300, 400, 1300, 400};
    boolean atwall = true; //used to know whether to run to or from
    boolean dup = false;
    boolean ddown = false;
    boolean dright = false;
    boolean dleft = false;
    boolean dslide = false;

    //distance vars
    double p = 0;   //position
    double t = -12; //target
    double s = 0; //speed
    double f = 141; //field size
    Trajectory t1;
    Trajectory t2;
    Trajectory t3;
    Trajectory f1;
    Trajectory f2;
    Trajectory f3;

    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        //Add Motors
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


        //Set Motors

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

        S0.setPosition(0.0);

        waitForStart();
        target = 200;
        while (opModeIsActive()) {
            Drive();
            ServoClamp();
            Slide();
            Coordinates();
        }
    }

    public void ServoClamp() {
        //automatic clamping if within distance
        if (gamepad1.left_bumper) {
            S0.setPosition(0.05);
        }
        if ((target == 200) && (D1.getDistance(DistanceUnit.MM) <= 33) || gamepad1.right_bumper) {
            target = 0;
            while (Math.abs(target - 1.4*M0_2.getCurrentPosition()) > 10) {
                M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - 1.4*M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - 1.4*M0_2.getCurrentPosition()) / 250)))));
            }
            M0_2.setPower(0);
            S0.setPosition(0.25);
        }
    }

    public void Slide() {
        M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - 1.4*M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250)))));
        if (gamepad1.a) {
            target = 200;
            S0.setPosition(0.05);
        }
        if (gamepad1.b) target = 2550;
        if (gamepad1.y) target = 1950;
        if (gamepad1.x) target = 1300;
        if (gamepad2.a){
            S1.setPosition(0.75);
            S2.setPosition(0);
        }if (gamepad2.b){
            //down
            S1.setPosition(0);
            //up
            S2.setPosition(0.75);
        }
        if (D0.getState() && (target == 0)) {
            M0_2.setDirection(DcMotor.Direction.FORWARD);
            M0_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            M0_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            M0_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if ((!gamepad1.dpad_down) && dslide) {
            dslide = false;
            target = target - Sset;
        }
        if ((!gamepad1.dpad_down) && !dslide) {
            dslide = true;
            target = target + Sset;
        }
    }



    public void Drive() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
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

        if (gamepad1.right_stick_button) {
            if (atwall) {
                y = ycord;
                x = xcord;
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


                t1 = drive.trajectoryBuilder(new Pose2d(0,0,0))
                        .lineToLinearHeading(new Pose2d(vy, 0, Math.toRadians(0)))
                        .addDisplacementMarker(() -> drive.followTrajectoryAsync(t2))
                        .build();
                //move to x position
                t2 = drive.trajectoryBuilder(t1.end())
                        .lineToLinearHeading(new Pose2d(vy, vx, Math.toRadians(vo)))
                        .addDisplacementMarker(() -> drive.followTrajectoryAsync(t3))
                        .build();
                //move diagonal forwards to target junction
                t3 = drive.trajectoryBuilder(t2.end())
                        .forward(d)
                        .build();
                //move diagonal backwards to center of tile
                //f1 = drive.trajectoryBuilder(new Pose2d((vy - d*Math.sqrt(2)/2),(vx + d * Math.sin(Math.toRadians(vo))),Math.toRadians(vo)))


                S0.setPosition(.3);
                target = hdata[x + 5*(y-1)+2];

                drive.followTrajectoryAsync(t1);
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
                atwall = false;
            }
            else {
                
                f1 = drive.trajectoryBuilder(new Pose2d(0,0,0))
                        .back(d)
                        .addDisplacementMarker(() -> drive.followTrajectoryAsync(f2))
                        .build();
                //move to x position
                f2 = drive.trajectoryBuilder(f1.end())
                        .lineToLinearHeading(new Pose2d(Math.abs(vx)*Math.cos(Math.toRadians(vo))-d, -vx*Math.cos(Math.toRadians(vo)), Math.toRadians(-vo)))
                        .addDisplacementMarker(() -> {
                            target = 200;
                            drive.followTrajectoryAsync(f3);
                        })
                        .build();
                //move diagonal forwards to target junction
                f3 = drive.trajectoryBuilder(f2.end())
                        .lineToLinearHeading(new Pose2d(-vy*Math.cos(Math.toRadians(vo))-d + Math.abs(vx)*Math.cos(Math.toRadians(vo)), vy*Math.sin(Math.toRadians(vo))-vx*Math.cos(Math.toRadians(vo)), Math.toRadians(-vo)))
                        .build();
                S0.setPosition(0);
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
        if (gamepad2.dpad_up) dup = true;
        if (gamepad2.dpad_down) ddown = true;
        if (gamepad2.dpad_left) dleft = true;
        if (gamepad2.dpad_right) dright = true;
        if (gamepad2.right_bumper) {
            xi = .5;
        }
        if (gamepad2.left_bumper) {
            xi = -.5;
        }
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
        telemetry.addData("x", xcord);
        telemetry.addData("y", ycord);
        telemetry.addData("xi", xi);
        telemetry.addData("atwall", atwall);
        telemetry.addData("front", D1.getDistance(DistanceUnit.INCH));
        telemetry.addData("right", D2.getDistance(DistanceUnit.INCH));
        telemetry.addData("left", D4.getDistance(DistanceUnit.INCH));

        //telemetry.addData("right", D2.getDistance(DistanceUnit.MM));
        //telemetry.addData("left", D4.getDistance(DistanceUnit.MM));

        telemetry.update();

    }
}

