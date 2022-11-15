package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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

    //teleop vars
    TrajectorySequence traj;
    Pose2d currentpose;
    public static double d1 = 11.5;
    public static double d2 = 3;
    public static double Sdrop = 350;
    double target;
    boolean sidered = true;
    boolean yfirst;
    int y = 2;
    int x = 0;
    int w = 1;
    double vy;
    double vx;
    double d;
    double vo;
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
    int[] hdata = {400, 1300, 400, 1300, 400,
            1300, 1950, 2550, 1950, 1300,
            400, 2550, 400, 2550, 400,
            1300, 1950, 2550, 1950, 1300,
            400, 1300, 400, 1300, 400
            ,200,200,200,200,200,200,200,200,200,200,200};


    //auto vars
    double x5;
    double o5;
    double y5;
    double park;
    int[] xcord = new int[]{-2, -1, 0};
    int[] ycord = new int[]{2, 2, 2};
    double dback = 5;
    double dwall = 5;
    double dslam = 5;
    TrajectorySequence init1;
    TrajectorySequence init2;
    TrajectorySequence parktraj;
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
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.0, 16.0 / 12.0);
        }
        FtcDashboard.getInstance().startCameraStream(tfod, 0);
        while (!isStarted()) {
            IdentifyVuforia();
        }
        S0.setPosition(0);
        S1.setPosition(0);
        S2.setPosition(0.75);

        Init();
        Cycle();
        Park();
    }

    public void Init() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        target = 600;
        if (sidered) {
            x1 = 0;
            y1 = dback;
            o1 = Math.toRadians(180);

            x2 = -(D2.getDistance(DistanceUnit.INCH) - dwall);
            y2 = dback;
            o2 = Math.toRadians(-90);

            x3 = x2;
            y3 = -dslam;
            o3 = Math.toRadians(-90);

            x4 = x3;
           y4 = 0;
           o4 = Math.toRadians(-90);

           x5 = -(dslam + dwall);
           y5 = 53;
           o5 = Math.toRadians(-90);


        } else {
            x1 = 0;
            y1 = dback;
            o1 = Math.toRadians(0);

            x2 = (D4.getDistance(DistanceUnit.INCH) - dwall);
            y2 = dback;
            o2 = Math.toRadians(-90);

            x3 = x2;
            y3 = -dslam;
            o3 = Math.toRadians(-90);

            x4 = x3;
            y4 = 0;
            o4 = Math.toRadians(-90);

            x5 = dslam + dwall;
            y5 = 53;
            o5 = Math.toRadians(90);

        }
        drive.setPoseEstimate(new Pose2d());

        init1 = drive.trajectorySequenceBuilder(new Pose2d(0,0,o4))
                .splineToConstantHeading(new Vector2d(x1,y1),o1)
                .splineToConstantHeading(new Vector2d(x2,y2),o2)
                .splineToConstantHeading(new Vector2d(x3,y3),o3)
                //.lineTo(new Vector2d(x4,y4))
                .build();
        drive.followTrajectorySequenceAsync(init1);
       while( drive.isBusy()
                && !isStopRequested()){
           drive.update();
           Slide();
       }
       drive.setPoseEstimate(new Pose2d(x4,y4,o4));

        init2 = drive.trajectorySequenceBuilder(new Pose2d(0,0,o4))
                .back(y5)
                .turn(o5)
                .forward(x5)
                .build();

        drive.followTrajectorySequenceAsync(init2);
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
        target = 600;
        parktraj = drive.trajectorySequenceBuilder(currentpose)
                .back(d1)
                .lineToLinearHeading(new Pose2d(park,vy,0))
                .build();
        drive.followTrajectorySequenceAsync(parktraj);
        while( drive.isBusy()
                && !isStopRequested()){
            drive.update();
            Slide();
        }
    }


    public void Cycle(){
        for(int i = 0;i < xcord.length; i++){
            int xm;
            if (sidered){
                xm = 1;
            }else{
                xm = -1;
            }

            x = xm* xcord[i];
            y = ycord[i];
            ServoClamp();
            Drive();
            Drive();
        }
    }
    public void ServoClamp() {
        double prevtarget = target;
        target = 0;
        while (Math.abs(target - M0_2.getCurrentPosition()) > 10) {
            M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250)))));
        }
        M0_2.setPower(0);
        S0.setPosition(0.25);
        target = prevtarget;

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
            if(w == 3) {
                vy = 24 * (y - 3) - 12;
                if (x >= 1) {
                    vo = Math.toRadians(45);
                    vx = 24 * (x - 1) + 12;
                } else {
                    vo = Math.toRadians(135);
                    vx = 24 * x + 12;
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
            target = hdata[x + 5*(y-1)+2];
        }
        else {
            if(w == 1){
                ix = -65;
                iy = -12;
                io = Math.toRadians(180);
                target = 600;
                yfirst = true;

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
                yfirst = true;
            }


            if(yfirst){
                x3 = vx;
                y3 = iy;

            }
            else{
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
            S0.setPosition(0); //drop and up on umbrella
            S1.setPosition(0);
            S2.setPosition(0.7);

        }

        drive.setPoseEstimate(currentpose);

        traj = drive.trajectorySequenceBuilder(currentpose)
                .lineToLinearHeading(new Pose2d(x1 + 0.02,y1 + 0.02,o1))
                .addDisplacementMarker(() ->{
                    if (atwall){
                        slidecalibrated = false;
                    }
                    if (!atwall){
                        S1.setPosition(0.70);
                        S2.setPosition(0);
                    }
                })
                .lineToLinearHeading(new Pose2d(x2 + .01,y2 + .01,o2))
                .lineToLinearHeading(new Pose2d(x3 + .01,y3 + .03,o3))
                .lineToLinearHeading(new Pose2d(x4,y4,o4))
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
        }
        if (!atwall){
            target = target - Sdrop;
            S0.setPosition(0.05);
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
                telemetry.addLine("working");
                telemetry.update();
            }
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
}



