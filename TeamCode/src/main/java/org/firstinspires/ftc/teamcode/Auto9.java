package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
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

    public static double yoffset_ = 5;  //constant added to all y positions
    public static double d_ = 13;  //diagonal distance forward and backward
    public static double x1i_ = 4;
    public static double y1i_ = 51.5;
    public static double h_ = 600; //starting stack height
    public static boolean red_ = true;
    public static int cycles_ = 5;


    double yoffset = yoffset_;  //constant added to all y positions
    double d = d_;  //diagonal distance forward and backward
    double x1i = x1i_;
    double y1i = y1i_;
    double h = h_; //starting stack height
    boolean red = red_;
    int cycles = cycles_;

    int i;

    public static double xstart = 65;
    public static double ystart = 36;
    double ostart = 0;
    public static double y1 = 60;
    public static double y2 = 65;
    public static double x1 = 12;
    public static double slidei = 550;
    public static double slided = -50;

    public static boolean audience = false;
    double o1 = 90;
    double o2 = -135;
    int imax;

    double vy = 1;  //vector roadrunner x value
    double vx = 1;  //vector roadrunner y value
    double vo = 1;  //target roadrunner theta
    double xi = -.5;  //initial robot position against wall in coordinate system, either .5 or -.5

    double target;
    int x;
    int y;

    boolean atwall = true; //used to know whether to run to or from
    boolean slidecalibrated = false;
    boolean slidecalfiller = true;
    boolean audienceside = true;


    int loop = 0; //variable for number of cycles
    //distance vars
    double p = 0;   //position
    double t = -12; //target
    double s = 0; //speed
    double f = 141; //field size

    int zonenumber;

    int park = 10;
    Trajectory t1;
    Trajectory t2;
    Trajectory t3;
    Trajectory t4;
    Trajectory f1;
    Trajectory f2;
    Trajectory f3;
    Pose2d start = new Pose2d(xstart,ystart,ostart);
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
    int[] hdata = new int[]{200, 1100, 200, 1100, 200,
            1100, 1750, 2350, 1750, 1100,
            200, 2350, 200, 2350, 200,
            1100, 1750, 2350, 1750, 1100,
            200, 1100, 200, 1100, 200};
    int[] cords = new int[]{
            -1, 1,
            0, 1,
            -1, 2,
            0, 2,
            0, 2,
    };


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
        start = new Pose2d(xstart,ystart,ostart);
        Init();
        Cycle();
        Park();




    }
    public void Init(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
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
                S0.setPosition(0.3);
                target = 200;
                //raise cone off ground
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
                S0.setPosition(.3);
                target = 2350;
                //set the slide to high junction before moving
            }
            if (i == 1){

            }
            if (i == 2){
                target = 2000;
                UntilSlide();
                S0.setPosition(0);
                //put slide down so cup on junction. Wait for slide to finish then drop cone
            }
            if (i == 3){
                target = slidei;
                //set slide to top of stack height
            }
            if (i == 4){

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

                } else {
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

   public void Cycle(){
        while (loop < cycles){
            target = h - (i*loop)-300;
            //Center(); //cetner using distance sensors
            UntilSlide(); //way faster slide speed but will wait until value is hit
            S0.setPosition(0.3); //clamp
            target = h - i*loop ; //set slide to lift cone above stack
            UntilSlide();   //wait for slide to go up
            x = cords[2*loop];  //set new cords based on preset cords list
            y = cords[2*loop + 1];
            Drive(); //to
            S0.setPosition(0);
            if (loop < (cycles - 1))Drive(); //from if its not the last cycle
            loop += 1;
        }
    }
    public void Park(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Trajectory traj3 = drive.trajectoryBuilder(t1.end())
                .splineToConstantHeading(new Vector2d(vy, vx), Math.toRadians(vo))
                .splineToConstantHeading(new Vector2d(vy + ((xi/Math.abs(xi))*(24* (zonenumber-2) + .1)), vx), Math.toRadians(vo))  //if xi = .5 then add (zone-2)*24 to y   // if xi = -.5 then subtract (zone-2)*24 from y
                .build();
        drive.followTrajectoryAsync(traj3);
        drive.update();
        while (drive.isBusy()) {
            drive.update();
            Slide();
        }

    }
    public void UntilSlide() {
        if ((target - M0_2.getCurrentPosition()) > 0)
        {
            M0_2.setPower(.1);
            while (target > M0_2.getCurrentPosition()) {
                //* ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition())))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()))))));
            }
        }
        else{
            M0_2.setPower(-.1 );
            while (target < M0_2.getCurrentPosition()) {
                //* ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition())))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()))))));
            }
        }

        M0_2.setPower(0);
    }


    public void Center(){
        //d2 = right
        //d4 = left
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        p = (D2.getDistance(DistanceUnit.INCH) - (f - D4.getDistance(DistanceUnit.INCH)));
        Trajectory c1 = drive.trajectoryBuilder(new Pose2d(0,0,0))
                .lineToLinearHeading(new Pose2d(0, (24*xi)-p, Math.toRadians(0)))
                .build();
        drive.followTrajectoryAsync(c1);
        drive.update();
        while (drive.isBusy()) {
            drive.update();
            Slide();
        }

    }

    public void Drive() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        if (atwall) {
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
            target = hdata[x + 5*(y-1)+2];
            t1 = drive.trajectoryBuilder(new Pose2d(0,0,0))
                    .lineToLinearHeading(new Pose2d(vy, 0, Math.toRadians(0)))
                    .addDisplacementMarker(() -> drive.followTrajectoryAsync(t2))
                    .build();
            //move to x position
            t2 = drive.trajectoryBuilder(t1.end())
                    .lineToLinearHeading(new Pose2d(vy, vx, Math.toRadians(vo)))
                    .addDisplacementMarker(() -> drive.followTrajectoryAsync(t3))
                    .build();
            //move diagonal forwards to target junction
            t3 = drive.trajectoryBuilder(t2.end())
                    .forward(d)
                    .addDisplacementMarker(() -> drive.followTrajectoryAsync(f1))
                    .build();
            //move diagonal backwards to center of tile
            f1 = drive.trajectoryBuilder(t3.end())
                    .back(d)
                    .addDisplacementMarker(() -> drive.followTrajectoryAsync(f2))
                    .build();
            //move to 0 x and 0 theta
            f2 = drive.trajectoryBuilder(f1.end())
                    .lineToLinearHeading(new Pose2d(vy, 0, 0))
                    .addDisplacementMarker(() -> drive.followTrajectoryAsync(t3))
                    .build();
            //move to 0 y
            f3 = drive.trajectoryBuilder(f2.end())
                    .lineToLinearHeading(new Pose2d(0, 0, 0))
                    .build();

            drive.followTrajectoryAsync(t1);
            drive.update();
            while (drive.isBusy()) {
                drive.update();
                Slide();
            }
            atwall = false;

        }
        if (atwall == false) {
            S0.setPosition(0);
            target = 200;
            drive.followTrajectoryAsync(f1);
            drive.update();

            while (drive.isBusy()) {
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
