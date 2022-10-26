package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Disabled
@Config
@Autonomous(name="Auto0000", group="Linear Opmode")


public class Auto4 extends LinearOpMode {
    // Declare OpMode members.
    double target;
    DcMotor M0_2;
    Servo S0;
    DigitalChannel D1;
    ColorSensor C1;
    double loop = 0;
    public int zone = 0;
    FtcDashboard dashboard;
    public static double sensedistance = -19;
    public static double X1 = -51;
    public static double Y1 = -11;
    public static double X2 = -58.5;
    public static double Y2 = 27;
    public static double turncoef = 1;
    public static double slide1 = .27;
    public static double slide2 = -.3;
    public static double slide3 = -.4;
    public static double slide4 = .35;


    public static double slidei = 460;
    public static double slided = 70;



    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        S0 = hardwareMap.get(Servo.class,"S0");
        S0.setPosition(.3);
        C1 = hardwareMap.get(ColorSensor.class, "C1");
        M0_2 = hardwareMap.get(DcMotor.class, "M0_2");
        D1 = hardwareMap.get(DigitalChannel.class,"D1");

        M0_2.setDirection(DcMotor.Direction.FORWARD);
        M0_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M0_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M0_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Trajectory start1 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(X1, 0, Math.toRadians(
                        180)))
                .build();
        Trajectory start2 = drive.trajectoryBuilder(start1.end())
                .strafeLeft(-(Y1))
                .build();
        Trajectory loop1 = drive.trajectoryBuilder(start2.end())
                .forward(Math.abs(X2-X1))
                .build();
        Trajectory loop2 = drive.trajectoryBuilder(loop1.end())
                .back(Math.abs(X2-X1))

                .build();
        Trajectory loop3 = drive.trajectoryBuilder(loop2.end())
                .lineToLinearHeading(new Pose2d(X1, Y2, Math.toRadians(turncoef*    90)))
                .build();
        Trajectory loop35 = drive.trajectoryBuilder(loop3.end())
                .forward(1)
                .build();
        //
        Trajectory loop4 = drive.trajectoryBuilder(loop35.end())
                .lineToLinearHeading(new Pose2d(X1, Y1, Math.toRadians(180)))
                .build();

        Trajectory loopexit = drive.trajectoryBuilder(loop2.end())
                .strafeRight(9.5)
                .build();
        Trajectory loopexit2 = drive.trajectoryBuilder(loopexit.end())
                .back(24)
                .build();
        Trajectory park1 = drive.trajectoryBuilder(loopexit2.end())
                .strafeLeft(24)
                .build();
        Trajectory park3 = drive.trajectoryBuilder(loopexit2.end())
                .strafeRight(24)
                .build();



        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        if(isStopRequested()) return;

        M0_2.setPower(slide1);

        drive.followTrajectory(start1);
        drive.followTrajectory(start2);

        target = 2500;

        while (Math.abs(target - M0_2.getCurrentPosition()) > 10) {
            M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250)))));
        }

        M0_2.setPower(0);
        drive.followTrajectory(loop1);
        S0.setPosition(0.0);
        M0_2.setPower(slide2);
        drive.followTrajectory(loop2);
        while(loop < 2) {


            drive.followTrajectory(loop3);
            drive.followTrajectory(loop35);
            // set target before drop
            /*target = slidei - (slided*loop);
            while (Math.abs(target - M0_2.getCurrentPosition()) > 10) {
                M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250)))));
            }*/

            target = slidei - (slided*loop)-160;
            while (Math.abs((target - M0_2.getCurrentPosition())) > 10) {
                M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250)))));
            }/*
            target = M0_2.getCurrentPosition() - 150;
            while (Math.abs(target - M0_2.getCurrentPosition()) > 10) {
                M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250)))));
            }*/
            S0.setPosition(0.51);
            M0_2.setPower(slide4);
            while (M0_2.getCurrentPosition() < slidei - (slided*loop)){

            }
            drive.followTrajectory(loop4);

            target = 2500;
            while (Math.abs(target - M0_2.getCurrentPosition()) > 10) {
                M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250)))));
            }
            M0_2.setPower(0);
            drive.followTrajectory(loop1);
            S0.setPosition(0.0);
            M0_2.setPower(-.15);
            drive.followTrajectory(loop2);

            loop++;
        }
        target = 0;
        while (target - M0_2.getCurrentPosition() < 1) {
            M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250)))));
        }
        M0_2.setPower(0);
        drive.followTrajectory(loopexit);
        drive.followTrajectory(loopexit2);

        if (zone == 1){
            drive.followTrajectory(park1);
        }
        else if (zone == 2){
            //nothing
        }
        else if (zone == 3){
            drive.followTrajectory(park3);
        }
        //wile limit == false
        //drive forward
        //repeat

    }
}


