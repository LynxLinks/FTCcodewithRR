package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(1000);
       double d = 11.5;
        double Sdrop = 200;

        int y = 4;
        int x = 2;
        int w = 3;
        boolean atwall = true; //used to know whether to run to or from
        boolean yfirst = true;
        double target;
        double d1 = 11.5;
        double d2 = 5;
        double vy = 0;
        double vx = 0;
        double vo = 0;
        double ix = 0;
        double iy = 0;
        double io = 0;
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
        boolean dup;
        boolean ddown;
        boolean dright;
        boolean dleft;
        boolean dbright;
        boolean dbleft;
        boolean dslide;
        boolean slidecalibrated;
        boolean beenoff;
        Pose2d currentpose;
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
        int[] hdata = new int[]{400, 1300, 400, 1300, 400,
                1300, 1950, 2550, 1950, 1300,
                400, 1300, 400, 1300, 400,
                1300, 1950, 2550, 1950, 1300,
                400, 1300, 400, 1300, 400};






        ///////////////////////////
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
            //target = hdata[x + 5*(y-1)+2];
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


        }



        ///////////////////////////




        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(62, 50, Math.toRadians(320), Math.toRadians(210), 6.7)
                .followTrajectorySequence(drive ->

                        drive.trajectorySequenceBuilder(currentpose)
                                .splineToLinearHeading(new Pose2d(x1 + 0.02,y1 + 0.02,o1),Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(x2 + .01,y2 + .01,o2),o3)
                                .splineToLinearHeading(new Pose2d(x3 + .01,y3 + .03,o3),o3)
                                .splineToLinearHeading(new Pose2d(x4,y4,o4),o4)
                                /*
                                new Pose2d(-36,-65,Math.toRadians(-90))
                                .splineToConstantHeading(new Vector2d(-36,-60),Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-60,-60),Math.toRadians(-90))
                                .splineToConstantHeading(new Vector2d(-60,-65),Math.toRadians(-90))*/
                                .build());





                        /*.forward(30)
                                .turn(Math.toRadians(90))
                                .forward(30)
                                .turn(Math.toRadians(90))
                                .forward(30)
                                .turn(Math.toRadians(90))
                                .forward(30)
                                .turn(Math.toRadians(90))
                                .build()*/



        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)

                // Add both of our declared bot entities
                .addEntity(myFirstBot)


                .start();


    }

}