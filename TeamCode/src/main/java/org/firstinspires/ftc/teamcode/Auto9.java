package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
    double xstart = 65;
    double ystart = 36;
    int park = 10; 
    int [][] cord = {
            {32,46,180},
            {34,46,180}
    };
    boolean audience = false;
    boolean red = true;

    //
    Pose2d start = new Pose2d(0,0,0);
    //

    //Road Runner Variables




    //Dashboard Variables
    FtcDashboard dashboard;

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
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        if (audience){
            if (red) {
                start = new Pose2d(xstart, -ystart, 0);
            }
            else{
                start = new Pose2d(-xstart, -ystart, 180);
            }
        }
        else{
            if (red){
                start = new Pose2d(xstart, ystart, 0);
            }
            else{
                start = new Pose2d(-xstart, ystart, 180);
            }
        }
        for( int i = 0; i < 10;i++) {
            int istuff = i;
            Trajectory main = drive.trajectoryBuilder(start)
                    .lineToLinearHeading(new Pose2d(cord[i][0], cord[i][1],Math.toRadians(cord[i][2])))
                    .addDisplacementMarker(() -> {
                        if(istuff == 1) {
                           //clamp
                           //sleep for clamp
                        }
                        if(istuff == 1) {
                            //slide target
                        }

                    })
                    .build();
            start = main.end();
            drive.followTrajectoryAsync(main);
            drive.update();
            while (drive.isBusy()) {
                //slide stuff
                drive.update();

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
