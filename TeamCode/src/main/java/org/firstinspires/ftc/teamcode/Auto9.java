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
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

@Autonomous(name="Auto9", group="Linear Opmode")

public class Auto9 extends LinearOpMode {
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
    public static double xstart = 65;
    public static double ystart = 36;
    double ostart = 0;
    public static double y1 = 60;
    public static double y2 = 65;
    public static double x1 = 12;
    public static double d = 11.5;
    public static double slidei = 550;
    public static double slided = -50;
    double o1 = 90;
    double o2 = -135;
    int i = 0;
    int imax;

    double target;

    boolean atwall = true; //used to know whether to run to or from
    boolean slidecalibrated = false;
    boolean slidecalfiller = true;
    boolean audienceside = true;

    int park = 10;
    double [][] cord;
    double [][] audiencecords = {

            {xstart,y1,ostart},
            {x1,y1,ostart},
            {x1,y2,o1},
            //absolute value of teleop cords

            {1,1},
            {1,2},
            {1,3},
            {0,3}

    };
    double [][] nonaudiencecords = {
            // absoluate balue roadrunner cords
            {x1,ystart,o1},
            {x1-(Math.sqrt(2)*d),ystart-(Math.sqrt(2)*d),o1},
            {x1,ystart,o1},
            {x1,y2,o2},
            //absolute value of teleop cords
            {1,3},
            {0,3},
            {0,2},
            {0,1}
    };
    public boolean audience = false;
    public boolean red = true;

    //

    //

    //Road Runner Variables




    //Dashboard Variables


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
        //RR import

        if (audience){
            imax = 3;
            cord = nonaudiencecords;
            if (red) {
                ystart = -ystart;
            }
            else{
                xstart = -xstart;
                ystart = -ystart;
                ostart = 180;
            }
        }
        else{
            imax = 4;
            cord = nonaudiencecords;
            if (red){


            }
            else{

                xstart = -xstart;
                ostart = 180;

            }
        }
        Pose2d start = new Pose2d(xstart,ystart,ostart);

        Actions();
        while(i<=imax) {
            i += 1;
            Trajectory main = drive.trajectoryBuilder(start)
                    .lineToLinearHeading(new Pose2d((xstart/Math.abs(xstart))*cord[i][0], (ystart/Math.abs(ystart))*cord[i][1],(xstart/Math.abs(xstart))*Math.toRadians(cord[i][2])))
                    .addDisplacementMarker(() -> {
                        Actions();
                    })
                    .build();
            start = main.end();
            drive.followTrajectoryAsync(main);
            drive.update();
            while (drive.isBusy()) {
                drive.update();
                Slide();

            }

        }


    }
    public void Actions(){
        if (audience){
            if(i == 0){
                target = 200;
            }
            if (i == 1){
                S0.setPosition(0);
                target = slidei;
            }
            if (i == 2){

            }
            if (i == 3){

            }

        }
        else {
            if (i == 0){
                target = 2350;
            }
            if (i == 1){

            }
            if (i == 2){
                S0.setPosition(0);
            }
            if (i == 3){
                target = slidei;
            }
            if (i == 4){

            }
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
        if (!slidecalibrated) {
            if (D0.getState() == true && slidecalfiller) {
                M0_2.setPower(.3);
            }
            if (D0.getState() == false) {
                M0_2.setPower(-0.3);
                slidecalfiller = false;

            }
            if (D0.getState() == true && !slidecalfiller) {
                slidecalibrated = true;
                if (audienceside) {
                    target = 200;

                }
                else{
                    target = 2350;
                }
            }
        }


        M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250)))));

        if (D0.getState() && (target == 0)) {
            M0_2.setDirection(DcMotor.Direction.FORWARD);
            M0_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            M0_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            M0_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
