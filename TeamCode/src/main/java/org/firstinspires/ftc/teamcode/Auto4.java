package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="Auto0000", group="Linear Opmode")
@Config

public class Auto4 extends LinearOpMode {
    // Declare OpMode members.
    double target;
    DcMotor M0_2;
    Servo S0;
    DigitalChannel D1;
    ColorSensor C1;
    public static double X1 = -40;
    public static double Y1 = 8;
    double loop = 0;
    public int zone = 0;

    @Override
    public void runOpMode() {

        S0 = hardwareMap.get(Servo.class,"S0");
        S0.setPosition(.51);
        C1 = hardwareMap.get(ColorSensor.class, "C1");
        M0_2 = hardwareMap.get(DcMotor.class, "M0_2");
        D1 = hardwareMap.get(DigitalChannel.class,"D1");

        M0_2.setDirection(DcMotor.Direction.FORWARD);
        M0_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M0_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M0_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory start0 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(-20, 0, Math.toRadians(0)))
                .build();
        Trajectory start1 = drive.trajectoryBuilder(start0.end())
                .lineToLinearHeading(new Pose2d(-53, 0, Math.toRadians(180)))
                .build();
        Trajectory start2 = drive.trajectoryBuilder(start1.end())
                .strafeLeft(12.5)
                .build();
        Trajectory loop1 = drive.trajectoryBuilder(start2.end())
                .forward(7)
                .build();
        Trajectory loop2 = drive.trajectoryBuilder(loop1.end())
                .back(8)

                .build();
        Trajectory loop3 = drive.trajectoryBuilder(loop2.end())
                .lineToLinearHeading(new Pose2d(-53, 26, Math.toRadians(90)))
                .build();
        Trajectory loop4 = drive.trajectoryBuilder(loop3.end())
                .lineToLinearHeading(new Pose2d(-53, -13, Math.toRadians(180)))
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
        M0_2.setPower(.25);
        drive.followTrajectory(start0);
        if  (C1.red() < 900){
            telemetry.addData("1",C1.red());
            zone = 1;
        }
        if  ((900 < C1.red() ) & (C1.red() < 1600)){
            telemetry.addData("2",C1.red());
            zone = 2;
        }
        if  (C1.red() > 1600){
            telemetry.addData("1",C1.red());
            zone = 3;
        }


        drive.followTrajectory(start1);
        drive.followTrajectory(start2);


        target = 2500;
        while (Math.abs(target - M0_2.getCurrentPosition()) > 10) {
            M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250)))));
        }
        M0_2.setPower(0);
        drive.followTrajectory(loop1);
        S0.setPosition(0.0);
        M0_2.setPower(-.15);
        drive.followTrajectory(loop2);
        while(loop < 2) {


            drive.followTrajectory(loop3);

            target = 600;
            while (Math.abs(target - M0_2.getCurrentPosition()) > 10) {
                M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250)))));
            }

            while (D1.getState() == false) {
                M0_2.setPower(-.1);
            }
            target = M0_2.getCurrentPosition() - 150;
            while (Math.abs(target - M0_2.getCurrentPosition()) > 10) {
                M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250)))));
            }
            S0.setPosition(0.51);
            M0_2.setPower(.2);
            while (M0_2.getCurrentPosition() < 600){

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
