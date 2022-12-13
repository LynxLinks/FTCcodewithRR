package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.google.gson.annotations.Until;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;




@Config
@Autonomous(name="Auto12", group="Linear Opmode")

public class Auto12 extends LinearOpMode {

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

    //teleop vars
    TrajectorySequence traj;
    Pose2d currentpose;
    public static double d1 = 12.8;
    public static double d2 = 3;
    public static double Sdrop = 350;
    double target;
    public static boolean sidered = true;
    public static double vy;
    public static double vx;
    public static double vo;
    public static Pose2d autopose = new Pose2d();
    boolean yfirst;
    public static double bump = 250;
    public static double slideoffset = 950;
    public static double vopark;
    int y;
    int x;
    int w = 1;
    double starget;
    double d;
    double ix;
    double iy;
    double io;
    double x1;
    double x2;
    double x3;
    double x4;
    double y1;
    double y2;
    double y3;
    double y4;
    double o1;
    double o2;
    double o3;
    double o4;
    boolean atwall = true;
    boolean beenoff = false;
    boolean slidecalibrated = false;

    int[] hdata = {100, 1300, 100, 1300, 100,
            1300, 1950, 2200, 1950, 1300,
            100, 2200, 100, 2200, 100,
            1300, 1950, 2200, 1950, 1300,
            100, 1300, 100, 1300, 100
            ,200,200,200,200,200,200,200,200,200,200,200};


    //auto vars
    double x5;
    double o5;
    double y5;
    double park;
    int[] xcord = new int[]{-1};
    int[] ycord = new int[]{3};
    public static double dback = 0;
    public static double dwall = 3.5;
    public static double dwall2 = 9;
    public static double dslam = 7;
    public static double slidei = -150  ;
    public static double leftstack = 5;
    public static double rightstack = 5;
    public static double strafe = 1;
    public static double slidespeed = 0.4;
    public static double parkoffset = -5;
    int xm;

    TrajectorySequence init1;
    TrajectorySequence init2;
    TrajectorySequence parktraj;
    TrajectorySequence slam1;
    String zone = "3";


    //viewforia Variables
    private static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/tflitemodels/model2.tflite";
    private static final String[] LABELS = {
            "1",
            "2",
            "3"
    };
    private static final String VUFORIA_KEY =
            "AW9YHO7/////AAABmac9s9LxrUDQuPRw+qlK4+1ZyAszPO7ouIyCLjm98NVZSbtunzGw0u8sSmhuTWNKjpGUxGCkMqV1mVUxgl9h4/J0GxK6120V5SfAcPH2XO17MGzFAm421Lcixendmv2WpyNU3HqERp0Og+sWFVwQTMM5f9rPnzsGiOVSOJ3xgZ9vltV2yHIrYxq1X95szieyo3xGab+kyy4mP5gWgu4VYwqffKb+nhXQ28jTzYhkqTbmE1saub+9juGnkNbqolX3A82q6/jrpIq1a/Nx5Egebwv1ItuABv0lq0gQJ4MiAGOf6czB9FnreVCYxSA4bUvCEZYxUG9RPgbczmU6eW85/wDskT3+1vMYR+BoqqwSa0mr";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public void runOpMode() {

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
        D3 = hardwareMap.get(DistanceSensor.class, "D3");
        D4 = hardwareMap.get(DistanceSensor.class, "D4");
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
        M0_2.setDirection(DcMotor.Direction.FORWARD);
        M0_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M0_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M0_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        initVuforia();
        initTfod();
        if (D2.getDistance(DistanceUnit.INCH)<35){
            sidered = true;
            xm = 1;

        }
        else{
            sidered = false;
            xm = -1;
            w = 4;

        }

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.0, 16.0 / 12.0);
        }

        FtcDashboard.getInstance().startCameraStream(tfod, 0);

        while (!isStarted()) {
            IdentifyVuforia();
        }

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

        S0.setPosition(0);
        S1.setPosition(0);
        S2.setPosition(0.70);
        Init();
        Cycle();
        Park();

    }

    public void Init() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        target = 900;
        if (sidered) {


            x1 = -(D2.getDistance(DistanceUnit.INCH) - dwall);
            y1 = dback;
            o1 = Math.toRadians(-90);

            x2 =  -(D2.getDistance(DistanceUnit.INCH) - dwall2);
            y2 = 52;
            o2 = Math.toRadians(180);

            x3 = dwall2 + dslam;


        } else {
            x1 = (D4.getDistance(DistanceUnit.INCH) - dwall);
            y1 = dback;
            o1 = Math.toRadians(-90);

            x2 =  (D4.getDistance(DistanceUnit.INCH) - dwall2);
            y2 = 52;
            o2 = Math.toRadians(0);

            x3 = dwall2 + dslam;


        }
        drive.setPoseEstimate(new Pose2d(0,0,Math.toRadians(-90)));
        init1 = drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(x1,y1,o1))
                .lineToLinearHeading(new Pose2d(x2,y2,o2))
                .forward(x3)
                .build();

        drive.followTrajectorySequenceAsync(init1);
        drive.update();
        while( drive.isBusy()
                && !isStopRequested()){
            drive.update();
            Slide();
        }
    }


    public void Park() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        if (sidered) {
            if (zone == "1") {
                park = -60;
            }
            if (zone == "2") {
                park = -36;
            }
            if (zone == "3") {
                park = -12;
            }
        }else{
            if (zone == "1") {
                park = 12;
            }
            if (zone == "2") {
                park = 36;
            }
            if (zone == "3") {
                park = 60;
            }
        }

        if (sidered){
            vopark = Math.toRadians(180);
        }
        else{
            vopark = Math.toRadians(0);
        }
        drive.setPoseEstimate(new Pose2d(vx + d*Math.cos(vo),vy + d*Math.sin(vo),vo));
        target = 800;
        parktraj = drive.trajectorySequenceBuilder(new Pose2d(vx + d*Math.cos(vo),vy + d*Math.sin(vo),vo))
                .back(d1)
                 .lineToLinearHeading(new Pose2d(park-.01,vy-3,vopark))
                .build();
        drive.followTrajectorySequenceAsync(parktraj);
        vx = park;
        vy = vy -3;
        while( drive.isBusy()
                && !isStopRequested()){
            drive.update();
            Slide();
        }
        autopose = drive.getPoseEstimate();
    }

    public void Cycle(){

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        for(int i = 0;i < xcord.length; i++){

            if (sidered && i == 1){
                d1 = 14.8;
            }
            if (!sidered){
                d1 = 14.2;
            }
            x = xm * xcord[i];
            y = ycord[i];
            ServoClamp();
            Drive();

            if (i < xcord.length -1) {
                Drive();
                /*drive.setPoseEstimate(new Pose2d());
                slam1 = drive.trajectorySequenceBuilder(new Pose2d())
                        .forward(dslam)
                        //.strafeLeft(strafe)
                        .build();
                drive.followTrajectorySequenceAsync(slam1);
                drive.update();
                while (drive.isBusy()) {
                    drive.update();
                    Slide();
                }

                 */
            }

        }
    }
    public void ServoClamp() {
        S0.setPosition(0.21);
        M0_2.setPower(-.5);
        while (D5.getState() == false && M0_2.getCurrentPosition() > 0){
        }
        if (w == 1 || w ==4){
            S0.setPosition(.37);
        }
        else {
            S0.setPosition(.47);
        }
        //telemetry.addData("current",M0_2.getCurrentPosition());
        target = M0_2.getCurrentPosition() - bump;
        //telemetry.addData("target",target);
        //telemetry.update();
        UntilSlide();
        target = target + slideoffset;
        UntilSlide();
    }
    public void Drive() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        if (atwall) {
            if (y ==6){
                d = d2;
            }
            else{
                d = d1;
            }
            if(w == 1){
                if (y >= 3){


                    vy = 24 * (y - 3) -12;
                    if (x <= -2){
                        vx = 24 * (x+1) - 12;
                        vo = Math.toRadians(135);
                    }
                    else{


                        vx = 24 * x - 12;
                        vo = Math.toRadians(45);
                    }
                }
                else{
                    vy = 24 * (y - 2) -12;
                    if (x <= -2){
                        vx = 24 * (x+1) - 12;
                        vo = Math.toRadians(-135);
                    }
                    else{
                        vx = 24 * x - 12;
                        vo = Math.toRadians(-45);
                    }
                }
            }
            if(w == 2){

                if (x >= 0){

                    vx = 24 * x -12;
                    if (y <= 1){
                        vy = 24 * (y -2) - 12;
                        vo = Math.toRadians(-45);
                    }else{
                        vy = 24 * (y -3) - 12;
                        vo = Math.toRadians(45);
                    }
                }
                else{

                    vx = 24 * (x + 1) -12;
                    if (y <= 1){
                        vy = 24 * (y -2) - 12;
                        vo = Math.toRadians(-135);
                    }else{
                        vy = 24 * (y -3) - 12;
                        vo = Math.toRadians(135);
                    }
                }
            }
            if(w == 3) {
                if (x >= 1) {
                    vx = 24 * (x - 1) + 12;
                    if (y <= 1){
                        vy = 24 * (y -2) - 12;
                        vo = Math.toRadians(-45);
                    }else{
                        vy = 24 * (y -3) - 12;
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

            }
            if(w == 4){
                if (y >= 3){
                    vy = 24 * (y - 3) - 12;
                    if (x >= 2){
                        vx = 24 * (x-1) + 12;
                        vo = Math.toRadians(45);
                    }
                    else{
                        vx = 24 * x + 12;
                        vo = Math.toRadians(135);
                    }
                }
                else{
                    vy = 24 * (y - 2) -12;
                    if (x >= 2){
                        vx = 24 * (x-1) + 12;
                        vo = Math.toRadians(-45);
                    }
                    else{
                        vx = 24 * x + 12;
                        vo = Math.toRadians(-135);
                    }


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
            x3 = vx;
            y3 = vy;
            x4 = vx + d * Math.cos(vo);
            y4 = vy + d * Math.sin(vo) ;
            o1 = io;
            o2 = io;
            o3 = vo;
            o4 = vo;
            currentpose = new Pose2d(ix,iy,io);
            atwall = false;
            //target = hdata[x + 5*(y-1)+2];
        }
        else {
            if(w == 1){
                ix = -65;
                iy = -12;
                io = Math.toRadians(180);
                starget = 850;
                yfirst = true;

            }
            if(w == 2){
                ix = -12;
                iy = -65;
                io = Math.toRadians(-90);
                starget = 300;
            }
            if(w == 3){
                ix = 12;
                iy = -65;
                io = Math.toRadians(-90);
                starget = 300;
            }
            if(w == 4){
                ix = 66.5;
                iy = -12;
                io = 0;
                starget = 850;
                yfirst = true;
            }


            if(yfirst){
                x3 = vx;
                y3 = iy;

            } else{
                x3 = ix;
                y3 = vy;
            }

            x1 = vx;
            y1 = vy;
            x2 = vx;
            y2 = vy;
            x4 = ix;
            y4 = iy;
            o1 = vo;
            o2 = io;
            o3 = io;
            o4 = io;

            currentpose = new Pose2d(vx + d*Math.cos(vo),vy + d*Math.sin(vo),vo);
            atwall = true;

        }

        drive.setPoseEstimate(currentpose);

        traj = drive.trajectorySequenceBuilder(currentpose)
                .lineToLinearHeading(new Pose2d(x1 + 0.02,y1 + 0.02,o1))
                .addDisplacementMarker(() ->{
                    if (atwall){
                        slidecalibrated = false;
                        target = starget;
                    }
                })
                .lineToLinearHeading(new Pose2d(x2 + .01,y2 + .01,o2))
                .addDisplacementMarker(() ->{
                    if (!atwall){
                        target = hdata[x + 5*(y-1)+2];
                        if(hdata[x + 5*(y-1)+2] > 1000){
                            S1.setPosition(0.70);
                            S2.setPosition(0);
                        }

                    }
                })

                .lineToLinearHeading(new Pose2d(x3 + .01,y3 + .03,o3))
                .lineToLinearHeading(new Pose2d(x4,y4,o4))
                .build();

        drive.followTrajectorySequenceAsync(traj);
        drive.update();
        while (
                 drive.isBusy()
                && !isStopRequested()

               ) {
            telemetry.addData("target", target);
            telemetry.update();
            drive.update();
            Slide();
        }
        if (!atwall){
            drop();
        }
    }

    public void IdentifyVuforia(){
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Objects Detected", updatedRecognitions.size());
                // step through the list of recognitions and display image position/size information for each one
                // Note: "Image number" refers to the randomized image orientation/number
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData("GoTo", recognition.getLabel(), recognition.getConfidence() * 100 );
                    zone = recognition.getLabel();
                }
                telemetry.addData("zone",zone);
                //telemetry.addLine("working");
                telemetry.addData("sidered",sidered);
                telemetry.addData("", "");
                telemetry.addData("right", D2.getDistance(DistanceUnit.INCH));
                telemetry.addData("left", D4.getDistance(DistanceUnit.INCH));
                //telemetry.addData("xm",xm);
                telemetry.update();
            }
        }
    }
    public void Slide () {

        if (slidecalibrated) {
            M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - 1.4*M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - 1.4*M0_2.getCurrentPosition()) / 250)))));
        }

        else {
            if (D0.getState() == true && !beenoff) { //if slide is on limit swtich
                M0_2.setPower(.35);
            }
            if (D0.getState() == false) { //if slide is above limit
                M0_2.setPower(-0.35);
                beenoff = true;
            }
            if (D0.getState() == true && beenoff) { //if slide is on limit and calibratedM0_2.setDirection(DcMotor.Direction.FORWARD);
                M0_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                M0_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                M0_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                slidecalibrated = true;
            }
        }
    }
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.1f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 330;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
    public void UntilSlide() {
        if ((target - 1.4*M0_2.getCurrentPosition()) > 0) {
            M0_2.setPower(slidespeed);
            while (target > 1.4*M0_2.getCurrentPosition()) { //faster slide algo
                //* ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition())))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()))))));
            }
        }
        else{
            M0_2.setPower(-slidespeed );
            while (target < 1.4*M0_2.getCurrentPosition()) { //faster slide algo
                //* ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition())))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()))))));
            }
        }
        M0_2.setPower(0);
    }
    public void drop(){
        S0.setPosition(0);
        double prevt = target;
        target = target - Sdrop;
        while (Math.abs(target - 1.4*M0_2.getCurrentPosition()) > 10) {
            M0_2.setPower(-.5 * ((1 - Math.pow(10, ((target - 1.4*M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - 1.4*M0_2.getCurrentPosition()) / 250)))));
        }
        S0.setPosition(0);
        S1.setPosition(0);
        S2.setPosition(0.7);
        target = prevt;
        while (Math.abs(target - 1.4*M0_2.getCurrentPosition()) > 10) {
            M0_2.setPower(-.5 * ((1 - Math.pow(10, ((target - 1.4*M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - 1.4*M0_2.getCurrentPosition()) / 250)))));
        }
    }
}



