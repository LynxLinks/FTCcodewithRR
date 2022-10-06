package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="Auto0000", group="Linear Opmode")

public class Auto4 extends LinearOpMode {
    // Declare OpMode members.
    double target;
    DcMotor M0_2;
    Servo S0;
    DigitalChannel D1;
    public static double X1 = -40;
    public static double Y1 = 8;
    double loop = 0;

    @Override
    public void runOpMode() {

        S0 = hardwareMap.get(Servo.class,"S0");
        S0.setPosition(.51);

        M0_2 = hardwareMap.get(DcMotor.class, "M0_2");
        D1 = hardwareMap.get(DigitalChannel.class,"D1");

        M0_2.setDirection(DcMotor.Direction.FORWARD);
        M0_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M0_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M0_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Trajectory start1 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(51, 0, Math.toRadians(0)))
                .build();
        Trajectory start2 = drive.trajectoryBuilder(start1.end())
                .strafeLeft(10)
                .build();
        Trajectory loop1 = drive.trajectoryBuilder(start2.end())
                .forward(9)
                .build();
        Trajectory loop2 = drive.trajectoryBuilder(loop1.end())
                .back(9)

                .build();
        Trajectory loop3 = drive.trajectoryBuilder(loop2.end())
                .lineToLinearHeading(new Pose2d(51, -30, Math.toRadians(270)))
                .build();
        Trajectory loop4 = drive.trajectoryBuilder(loop3.end())
                .lineToLinearHeading(new Pose2d(51, 10, Math.toRadians(0)))
                .build();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        if(isStopRequested()) return;


        M0_2.setPower(0.40);
        drive.followTrajectory(start1);
        drive.followTrajectory(start2);
        while(loop < 6) {
            target = 2300;
            while (Math.abs(target - M0_2.getCurrentPosition()) > 10) {
                M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250)))));
            }
            M0_2.setPower(0);
            drive.followTrajectory(loop1);
            S0.setPosition(0.0);
            M0_2.setPower(-.15);
            drive.followTrajectory(loop2);
            drive.followTrajectory(loop3);

            target = 600;
            while (Math.abs(target - M0_2.getCurrentPosition()) > 10) {
                M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250)))));
            }

            while (D1.getState() == false) {
                M0_2.setPower(-.03);
            }
            target = M0_2.getCurrentPosition() - 160;
            while (Math.abs(target - M0_2.getCurrentPosition()) > 10) {
                M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250)))));
            }
            S0.setPosition(0.51);
            M0_2.setPower(.2);
            while (M0_2.getCurrentPosition() < 600){

            }
            drive.followTrajectory(loop4);
            loop++;
        }
        M0_2.setPower(0);

        //while limit == false
        //drive forward
        //repeat

    }
}
