package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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

@Autonomous(name="Auto10", group="Linear Opmode")

public class Auto10 extends LinearOpMode {
    //Variables
    String zone = "3";
    DcMotor M0;
    FtcDashboard dashboard;
    DcMotor M1;
    DcMotor M2;
    DcMotor M3;
    DcMotor M0_2;
    Servo S0;
    DigitalChannel D0;
    DistanceSensor D1;
    DistanceSensor D2;
    DistanceSensor D4;
    TrajectorySequence startmoves;
    public static double strafe1 = 19;
    public static double strafe2 = 19;
    public static double forward1 = 5;
    DistanceSensor D3; // back
    //public statics
    public static double d2 = 15; //distance strafe at pole but get interupted
    public static double d3 = 8; //distance come back off of pole
    public static double d4 = 3; //y offset when coming back
    public static double Sset = 200; //test drop distance
    public static double dxoffset = 2.5;
    public static double dyoffset = -.5;
    public static double yoffset = 6;
    public static int slamtime = 10;

    public static double dy2 = 2.5;
    public static double dx2 = -.5;
    public static double centertarget = 47;

    //constants
    double d; // distance to pole update from sensor
    double target; //slide target position
    boolean track = false; //update d yes or no
    int y;   // y finallized when built
    int x;   // x finallized when built
    int i;
    double multiplier;
    boolean fillerbool = true;
    double xi = -.5;  //initial robot position against wall in coordinate system, either .5 or -.5
    double vy;  //vector roadrunner x value
    double vx;  //vector roadrunner y value
    double vo;  //target roadrunner theta
    boolean atwall = true; //used to know whether to run to or from
    int ycord = 2; // live cord y
    int xcord = 0; // live cord x
    boolean drop = false;
    int[] hdata = new int[]{200, 1100, 200, 1100, 200,
            1100, 1750, 2350, 1750, 1100,
            200, 2350, 200, 2350, 200,
            1100, 1750, 2350, 1750, 1100,
            200, 1100, 200, 1100, 200}; //slide heights
    Trajectory t1;
    Trajectory t2;
    Trajectory t4;
    TrajectorySequence f15;
    Pose2d pole;


    //Slide variables
    Pose2d start;
    double movement;
    double park;
    double ymult = 1;
    double [][]cords = {

            {},
            {},
            {},

    };

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
        D0 = hardwareMap.get(DigitalChannel.class, "D0");
        D1 = hardwareMap.get(DistanceSensor.class, "D1");
        D2 = hardwareMap.get(DistanceSensor.class, "D2");
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
        if(zone == "1"){
            park = 15;
        }
        if(zone == "2"){
            park = 30;
        }
        if(zone == "3"){
            park = 45;
        }

        if(isStopRequested()) return;
        drive.setPoseEstimate(new Pose2d());
        startmoves = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeLeft(strafe1)
                .addDisplacementMarker(() -> {
                    target = 200;
                })
                .turn(Math.toRadians(90))
                .strafeLeft(strafe2)
                .forward(forward1)
                .build();

        drive.followTrajectorySequenceAsync(startmoves);
        drive.update();
        while(drive.isBusy()){

        }
        //RR import

        Actions();

        for(int i = 0;i < 5; i++) {
            movement = i;
            Actions();

            //going to pole movement

            track = false;
            dx2 = Math.abs(dx2);
            //set varibles based off of cordinates
            cords[i][2] = ycord;
            cords[i][1] = xcord;
            vy = yoffset + 24 * (y - 1);
            d2 = Math.abs(d2);
            //d2 = Math.abs(d2);
            if (x > 0) {
                vx =  24 * Math.floor(Math.abs(x - xi));
            } else {
                vx =  -24 * Math.floor(Math.abs(x - xi));
            }
            if (x > xi) {
                vo = 90;
            } else {
                vo = -90;
                if(vx != 0){
                    dx2 = - dx2;
                }
            }


            //build non x translation
            if(vx == 0){
                t1 = drive.trajectoryBuilder(new Pose2d(0,0,0))
                        .back(vy + d2)
                        .addDisplacementMarker(Math.abs(vy),() ->{
                            track = true;
                        })
                        .build();
            }

            //build with x translation
            else{
                d2 = Math.abs(d2);
                vo = 180;
                if (x < xi){
                    d2 = -d2;
                }
                t1 = drive.trajectoryBuilder(new Pose2d(0,0,0))
                        .back(vy)
                        .addDisplacementMarker(() -> {
                            drive.followTrajectoryAsync(t2);
                        })
                        .build();
                t2 = drive.trajectoryBuilder(t1.end())
                        .strafeLeft(vx+d2)
                        .addDisplacementMarker(Math.abs(vx),() ->{
                            track = true;
                        })
                        .build();
            }

            //close clamp
            S0.setPosition(.3);

            //retrieve height of pole
            target = hdata[x + 5*(y-1)+2];

            //movement 1 with distance interrupt
            drive.followTrajectoryAsync(t1);
            drive.update();
            while (Math.abs(gamepad1.left_stick_x) < .5
                    && Math.abs(gamepad1.left_stick_y) < .5
                    && Math.abs(gamepad1.right_stick_x) < .5
                    && Math.abs(gamepad1.right_stick_y) < .5
                    && drive.isBusy()
            ) {
                if(D4.getDistance(DistanceUnit.INCH) <= 10 && D4.getDistance(DistanceUnit.INCH) >=1 && track && vo == 90){
                    d = D4.getDistance(DistanceUnit.INCH);
                    break;
                }
                if(D2.getDistance(DistanceUnit.INCH) <= 10 && D2.getDistance(DistanceUnit.INCH) >=1 && track && vo == -90){
                    d = D2.getDistance(DistanceUnit.INCH);
                    break;
                }
                if(D3.getDistance(DistanceUnit.INCH) <= 10 && D3.getDistance(DistanceUnit.INCH) >=1 && track && vo == 180){
                    d = D3.getDistance(DistanceUnit.INCH);
                    break;
                }
                drive.update();
                Slide();
                while (gamepad1.right_stick_button){
                    M0.setPower(0);
                    M3.setPower(0);
                    M1.setPower(0);
                    M2.setPower(0);
                    Slide();
                }
            }

            //movement 2 build
            drive.setPoseEstimate(new Pose2d());
            if(d >= 10 || d <= 1){
                d = 5;
            }
            if(vx == 0){
                t4 = drive.trajectoryBuilder(new Pose2d(dyoffset,(dxoffset)*Math.sin(Math.toRadians(vo)),Math.toRadians(vo)))
                        .lineToLinearHeading(new Pose2d(dyoffset,(d+dxoffset)*Math.sin(Math.toRadians(vo)),Math.toRadians(vo)))
                        .build();
            }
            else{
                t4 = drive.trajectoryBuilder(new Pose2d(-dy2,dx2,Math.toRadians(vo)))
                        .lineToLinearHeading(new Pose2d(-(d +dy2),dx2,Math.toRadians(vo)))
                        .build();
            }

            //movement 2 onto pole
            drive.followTrajectoryAsync(t4);
            drive.update();
            while (Math.abs(gamepad1.left_stick_x) < .5
                    && Math.abs(gamepad1.left_stick_y) < .5
                    && Math.abs(gamepad1.right_stick_x) < .5
                    && Math.abs(gamepad1.right_stick_y) < .5
                    && drive.isBusy()
            ) {
                drive.update();
                Slide();
                while (gamepad1.right_stick_button){
                    M0.setPower(0);
                    M3.setPower(0);
                    M1.setPower(0);
                    M2.setPower(0);
                    Slide();
                }
            }
            target = target - Sset;
            drop = true;
            atwall = false;

             //movement returning to wall

            S0.setPosition(0);
            //neeeeeeds to be fixed but build back
            if(vx == 0) {
                pole = new Pose2d(-vy - 12,vx + Math.sin(Math.toRadians(vo))*d3 ,Math.toRadians(vo));
                drive.setPoseEstimate(pole);
                f15 = drive.trajectorySequenceBuilder(pole)
                        .back(d3)
                        .addDisplacementMarker(() -> {
                            target = 200;
                        })
                        .turn(Math.toRadians(-vo))
                        .forward(vy + d4 + 12)
                        .build();
            }
            else{
                pole = new Pose2d(-vy - 8,vx + ((12)*(x-xi)/Math.abs(x-xi)),Math.toRadians(vo));
                drive.setPoseEstimate(pole);
                f15 = drive.trajectorySequenceBuilder(pole)
                        .back(d3)
                        .addDisplacementMarker(() -> {
                            target = 200;
                        })
                        .turn(Math.toRadians(-vo))
                        .strafeRight(vx + 12)
                        .forward(vy + d4)
                        .build();

            }

            //open cam
            S0.setPosition(0);

            //return movement
            drive.followTrajectorySequenceAsync(f15);

            drive.update();
            while (Math.abs(gamepad1.left_stick_x) < .5
                    && Math.abs(gamepad1.left_stick_y) < .5
                    && Math.abs(gamepad1.right_stick_x) < .5
                    && Math.abs(gamepad1.right_stick_y) < .5
                    && drive.isBusy()
            )
            {
                drive.update();
                Slide();
                while (gamepad1.right_stick_button){
                    M0.setPower(0);
                    M3.setPower(0);
                    M1.setPower(0);
                    M2.setPower(0);
                    Slide();
                }
            }
            atwall = true;

        }


    }
    public void Actions(){
            if(movement == 0){
                target = 200;
            }
            if (movement == 1){
                S0.setPosition(0);
                target = 1000;
            }
            if (movement == 2){

            }
            if (movement == 3){

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
                telemetry.addLine("working");
                telemetry.update();
            }
        }
    }
    public void Slide () {
        M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250)))));
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
