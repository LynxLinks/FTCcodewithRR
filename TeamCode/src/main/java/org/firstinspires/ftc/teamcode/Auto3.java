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

@Autonomous(name="Auto3", group="Linear Opmode")
public class Auto3 extends LinearOpMode {
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
                .forward(90)
                .build();

        Trajectory to1 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(X1, 0, Math.toRadians(90)))
                .build();
        Trajectory to2 = drive.trajectoryBuilder(to1.end())
                .forward(Y1)
                .build();
        Trajectory from1 = drive.trajectoryBuilder(to2.end())
                .back(Y1)
                .build();
        Trajectory from2 = drive.trajectoryBuilder(from1.end())
                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)))
                .build();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        if(isStopRequested()) return;

        target = 2400;
        M0_2.setPower(0.55);
        drive.followTrajectory(to1);
        while (Math.abs(target - M0_2.getCurrentPosition()) > 10) {
            M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250)))));
        }
        M0_2.setPower(0);
        drive.followTrajectory(to2);
        S0.setPosition(0.0);
        drive.followTrajectory(from1);
        M0_2.setPower(-.4);
        drive.followTrajectory(from2);
        M0_2.setPower(-.1);
        //while limit == false
        //drive forward
        //repeat

    }
}
