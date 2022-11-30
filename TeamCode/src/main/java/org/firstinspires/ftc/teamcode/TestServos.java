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

public class TestServos extends LinearOpMode {

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

    public static double camBothClosed = 0.08;
    public static double camTopOpen= 0.27;
    public static double camBothOpen= 0.37;
    public static double UmbrellaMin1 = 0.02;
    public static double UmbrellaMin2 = 0.03;
    public static double UmbrellaMax1 = 0.7;
    public static double UmbrellaMax2 = 0.7;

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
            M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - 1.4*M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - 1.4*M0_2.getCurrentPosition()) / 250)))));
        } else {
            if (D0.getState() == true && !beenoff) { //if slide is on limit swtich
                M0_2.setPower(.4);
            }
            if (D0.getState() == false) { //if slide is above limit
                M0_2.setPower(-0.4);
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
        //autoservo
        /*if ((target <= 850) && (D1.getDistance(DistanceUnit.MM) <= 33)) {
           ServoClamp();
        }

         */
        //Manual Servo
        if (gamepad1.dpad_up) {
            S1.setPosition(UmbrellaMin1); //.02
            S2.setPosition(UmbrellaMax2); ;//.7
            S1.setPosition(0.02); //.02
            S2.setPosition(.7); ;//.7
        }
        if (gamepad1.dpad_down){
            S1.setPosition(UmbrellaMax1); //.7
            S2.setPosition(UmbrellaMin2); //.03
            S1.setPosition(.7); //.02
            S2.setPosition(.03); ;//.7
        }

        //Manual Slide
        if (gamepad2.y) S0.setPosition(camBothClosed);
        if (gamepad2.b) S0.setPosition(camTopOpen);
        if (gamepad2.a) S0.setPosition(camBothOpen);

        if (gamepad1.a) target = 500;
        if (gamepad1.b) target = 2550;
        if (gamepad1.y) target = 1950;
        if (gamepad1.x) target = 1300;

        if (slidecalibrated && gamepad1.left_bumper) {
            slidecalibrated = false;
        }
    }
}
