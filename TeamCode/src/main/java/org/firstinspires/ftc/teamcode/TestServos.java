package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.google.gson.annotations.Until;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.teamcode.Auto12.sidered;
import static org.firstinspires.ftc.teamcode.Auto12.slidespeed;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

//name and class

@Config
@TeleOp(name = "TestServos", group="Linear Opmode")

public class    TestServos extends LinearOpMode {

    DigitalChannel D5;
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

    public static double camBothClosed = 0.18;
    public static double camTopOpen= 0.37;
    public static double camBothOpen= 0.47;
    public static double UmbrellaMin1 = 0.02;
    public static double UmbrellaMin2 = 0.03;
    public static double UmbrellaMax1 = 0.7;
    public static double UmbrellaMax2 = 0.7;
    public static double zero = 0;
    public static double coneheight = 500;
    public static double low = 1300;
    public static double medium = 1900;
    public static double high = 2450;
    public static double bump = 250;
    public static double slideoffset = 950;
    boolean slidecalibrated = true;
    boolean beenoff = false;

    double target;

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
        waitForStart();

        while (opModeIsActive()) {
            Slide();
            UI();
        }

    }
    public void Slide () {

        if (slidecalibrated) {
            M0_2.setPower(-1 * ((1 - Math.pow(5, ((target - 1.4*M0_2.getCurrentPosition()) / 500))) / (1 + Math.pow(5, ((target - 1.4*M0_2.getCurrentPosition()) / 500)))));
        } else {
            if (D0.getState() == true && !beenoff) { //if slide is on limit swtich
                M0_2.setPower(.2);
            }
            if (D0.getState() == false) { //if slide is above limit
                M0_2.setPower(-0.2);
                beenoff = true;
            }
            if (D0.getState() == true && beenoff) { //if slide is on limit and calibratedM0_2.setDirection(DcMotor.Direction.FORWARD);
                M0_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                M0_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                M0_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                slidecalibrated = true;
                beenoff = false;
                M0_2.setPower(0);
            }
        }
    }
    public void UI() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //autoservo
        /*if ((target <= 850) && (D1.getDistance(DistanceUnit.MM) <= 33)) {
           ServoClamp();
        }

         */
        //Manual Servo
        //test
        if (gamepad1.dpad_right){
            TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(-24, 0,Math.toRadians(180)))
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
        }
        if (gamepad1.dpad_up) {
            S1.setPosition(UmbrellaMin1); //.02
            S2.setPosition(UmbrellaMax2); ;//.7
        }
        if (gamepad1.dpad_down){
            S1.setPosition(UmbrellaMax1); //.7
            S2.setPosition(UmbrellaMin2); //.03
        }
        if(gamepad2.right_bumper){
            S0.setPosition(0.21);
            M0_2.setPower(-.5);
            while (D5.getState() == false && M0_2.getCurrentPosition() > 0){
            }
            S0.setPosition(.47);
            //telemetry.addData("current",M0_2.getCurrentPosition());
            target = M0_2.getCurrentPosition() - bump;
            //telemetry.addData("target",target);
            //telemetry.update();
            UntilSlide();
            target = target + slideoffset;
            UntilSlide();
        }

        //Manual Slide
        if (gamepad2.y) S0.setPosition(camBothClosed);
        if (gamepad2.b) S0.setPosition(camTopOpen);
        if (gamepad2.a) S0.setPosition(camBothOpen);

        if (gamepad1.a) target = coneheight;
        if (gamepad1.b) target = high;
        if (gamepad1.y) target = medium;
        if (gamepad1.x) target = low;
        if (gamepad1.right_bumper) target = zero;

        if (slidecalibrated && gamepad1.left_bumper) {
            slidecalibrated = false;
        }

        /*telemetry.addData("front", D1.getDistance(DistanceUnit.INCH));
        telemetry.addData("right", D2.getDistance(DistanceUnit.INCH));
        telemetry.addData("back", D3.getDistance(DistanceUnit.INCH));
        telemetry.addData("left", D4.getDistance(DistanceUnit.INCH));
        telemetry.addData("target", target);
        telemetry.addData("encoder", M0_2.getCurrentPosition());
        telemetry.update();

         */
    }
    public void UntilSlide() {
        if ((target - 1.4*M0_2.getCurrentPosition()) > 0) {
            M0_2.setPower(Auto13.slidespeed);
            while (target > 1.4*M0_2.getCurrentPosition()) ;
        }
        else{
            M0_2.setPower(-Auto13.slidespeed );
            while (target < 1.4*M0_2.getCurrentPosition());
        }
        M0_2.setPower(0);
    }
}
