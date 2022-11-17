package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

import java.util.List;

@Disabled
@Config
@Autonomous(name="Auto11", group="Linear Opmode")

public class Auto11 extends LinearOpMode {

    boolean sidered = true;
    double angel = 90;

    //Variables
    String zone = "3";
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
    DistanceSensor D4;
    TrajectorySequence startmoves;
    TrajectorySequence startmove2;
    public static double dwall = 4;
    public static double dwall2 = 2;
    public static double strafe2 = -53;
    public static double strafe3 = 12;
    public static double forward1 = 15;
    public static double slidei = 600;
    public static double slided = 100;
    public static double slidex = -350;

    DistanceSensor D3; // back
    //public statics
    public static double d2 = 15; //distance strafe at pole but get interupted
    public static double d3 = 8; //distance come back off of pole
    public static double d4 = 3; //y offset when coming back
    public static double Sset = 200; //test drop distance
    public static double dxoffset = 3.2;
    public static double dyoffset = -1.5;
    public static double yoffset = 6;
    public static int slamtime = 10;
    public static double slidespeed = .3;
    double strafe1;
    public static double dy2 = 2.5;
    public static double dx2 = -.5;
    public static double forward0 = 8;

    //constants
    double d = 11.5; // distance to pole update from sensor
    double target; //slide target position
    boolean track = false; //update d yes or no
    int y;   // y finallized when built
    int x;   // x finallized when built
    double xi = .5;  //initial robot position against wall in coordinate system, either .5 or -.5
    double vy;  //vector roadrunner x value
    double vx;  //vector roadrunner y value
    double vo;  //target roadrunner theta
    boolean atwall = true; //used to know whether to run to or from
    boolean beenoff = false;
    boolean slidecalibrated = false;
    int ycord = 2; // live cord y
    int xcord = 0; // live cord x
    boolean drop = false;
    int[] hdata = new int[]{400, 1300, 400, 1300, 400,
            1300, 1950, 2550, 1950, 1300,
            400, 1300, 400, 1300, 400,
            1300, 1950, 2550, 1950, 1300,
            400, 1300, 400, 1300, 400}; //slide heights
    TrajectorySequence t1;
    Trajectory t2;
    Trajectory t3;
    TrajectorySequence f1;
    Pose2d pole;


    //Slide variables
    Pose2d start;
    double movement;
    double park;
    double ymult = 1;
    int []cordY = {1,2};
    int []cordx = {1,1};
    int xmult = 1;
    double xcorrect = 5;
    double xcorrectback = 12;
    double parktrue = 0;


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
        //Sync Dashboard
        dashboard = FtcDashboard.getInstance();

        //start viewforia
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.0, 16.0/12.0);
        }

        //stream viewforia Camera to dashboard
        FtcDashboard.getInstance().startCameraStream(tfod, 0);

        //init loop
        while(!isStarted()){
            //detects and save zone
            IdentifyVuforia();
        }

        //up
        S1.setPosition(0);
        //up
        S2.setPosition(0.75);

        if(zone == "1"){
            park = -8;
        }
        if(zone == "2"){
            park = -30;
        }
        if(zone == "3"){
            park = -56;
        }
        if(sidered){
            strafe1 = (D2.getDistance(DistanceUnit.INCH) - dwall);
            angel = -90;
        }
        else{
            strafe1 = -(D4.getDistance(DistanceUnit.INCH) - dwall);
            angel = 90;
            xmult = -1;
            xi = -.5;

        }

        if(isStopRequested()) return;

        drive.setPoseEstimate(new Pose2d());
        startmoves = drive.trajectorySequenceBuilder(new Pose2d())
                .back(2)
                .strafeRight(strafe1)
                .forward(forward0)

                .build();

        startmove2 = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(strafe2,dwall2,0))
                .addDisplacementMarker(() -> {
                    //xcorrect = D1.getDistance(DistanceUnit.INCH);
                    target = slidei;
                })
                // .back(xcorrectback - xcorrect)
                // .strafeLeft(strafe3)
                .turn(Math.toRadians(angel))
                .forward(forward1)
                .build();

        drive.followTrajectorySequenceAsync(startmoves);
        drive.update();
        while(drive.isBusy()&&!isStopRequested()){
            drive.update();
            Slide();
        }
        drive.setPoseEstimate(new Pose2d());
        drive.followTrajectorySequenceAsync(startmove2);
        drive.update();
        while(drive.isBusy()&&!isStopRequested()){
            drive.update();
            Slide();
        }

        //RR import

        for(int i = 0;i < 2; i++) {
            if(i == 1){
                parktrue = park;
            }

            S0.setPosition(0);
            target = slidei - (slided*i)+slidex; //set slide to level to grab top cone
            UntilSlide(); //wait until slide height is hit
            S0.setPosition(0.3); //clamp
            target = slidei - (slided*i) +100; //set slide to lift cone above stack
            UntilSlide();
            track = false;
            //set varibles based off of cordinates
            y = cordY[i];
            x = xmult * cordx[i];
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

            drive.setPoseEstimate(new Pose2d());
            t1 = drive.trajectorySequenceBuilder(new Pose2d(0,0,0))
                    .lineToLinearHeading(new Pose2d(vy, 0, Math.toRadians(0)))
                    .turn(Math.toRadians(vo))
                    .addDisplacementMarker(() -> {
                        target = hdata[x + 5*(y-1)+2];
                        S1.setPosition(.75);
                        S2.setPosition(0);
                    })
                    .forward(d)
                    .addDisplacementMarker(() -> {
                        target = target - Sset;
                        drop = true;
                        atwall = false;
                        S0.setPosition(0);
                    })
                    .build();

            drive.followTrajectorySequenceAsync(t1);
            drive.update();
            while (drive.isBusy() && !isStarted()) {
                drive.update();
                Slide();
            }

            atwall = false;

            drive.setPoseEstimate(new Pose2d());
            f1 = drive.trajectorySequenceBuilder(new Pose2d(0,0,0))
                    .back(d)
                    .turn(Math.toRadians(-vo))
                    .addDisplacementMarker(() -> {
                        target = slidei;
                        S1.setPosition(0);
                        //up
                        S2.setPosition(0.75);
                    })
                    .lineToLinearHeading(new Pose2d(-vy*Math.cos(Math.toRadians(vo))-d + Math.abs(vx)*Math.cos(Math.toRadians(vo)), vy*Math.sin(Math.toRadians(vo))-vx*Math.cos(Math.toRadians(vo)), Math.toRadians(-vo)))
                    .build();
            drive.followTrajectorySequenceAsync(f1);
            drive.update();
            while (drive.isBusy() && !isStarted()) {
                drive.update();
                Slide();
            }
            atwall = true;

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
    public void UntilSlide() {
        if ((target - M0_2.getCurrentPosition()) > 0)
        {
            M0_2.setPower(slidespeed);
            while (target > M0_2.getCurrentPosition()) { //faster slide algo
                //* ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition())))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()))))));
            }
        }
        else{
            M0_2.setPower(-slidespeed );
            while (target < M0_2.getCurrentPosition()) { //faster slide algo
                //* ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition())))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()))))));
            }
        }
        M0_2.setPower(0);
    }
}
