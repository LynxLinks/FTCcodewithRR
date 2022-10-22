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
        double xoffset =         6;
        double yoffset = 5;  //constant added to all y positions
        double d = 12;
        double y1 =    53;
        double d4 =    29.3;
        int y = 5;   //y coordinate input
        int x = -2;   //x coordinate input
        double target; //slide target position
        double vy = 1;  //vector roadrunner x value
        double vx = 1;  //vector roadrunner y value
        double vo = 1;  //target roadrunner theta
        double xi = -.5;  //initial robot position against wall in coordinate system, either .5 or -.5

        double xstart = 36;
        double ystart = -65.3;
        double ostart = 270;
        vy = -(yoffset + 24 * (y - 1));
        if (x > 0) {
            vx = .1 - 24 * Math.floor(Math.abs(x - xi));
        } else {
            vx = .1 + 24 * Math.floor(Math.abs(x - xi));
        }
        if (x > xi) {
            vo = 135;
        } else {
            vo = -135;
        }

        // Declare our first bot
        double xwallstart = xstart + (d4);
        double ywallstart = ystart + y1;
        double owallstart = 0;
        double finalVy = ywallstart - vx;
        double finalVo = owallstart + vo;
        double finalVx = xwallstart + vy;

        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(30, 20, Math.toRadians(100), Math.toRadians(100), 6.7)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(xstart, ystart, Math.toRadians(ostart)))
                                .strafeLeft(d4 - xoffset)
                                .lineToLinearHeading(new Pose2d(xstart+(d4-xoffset), ystart+y1, Math.toRadians(0)))
                                .forward(xoffset)

                                .lineToLinearHeading(new Pose2d(finalVx,ywallstart , Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(finalVx, finalVy, Math.toRadians(finalVo)))
                                .lineToLinearHeading(new Pose2d(finalVx + (d * Math.sin(Math.toRadians(finalVo))), finalVy + (d * Math.cos(Math.toRadians(finalVo))), Math.toRadians(finalVo)))
                                .lineToLinearHeading(new Pose2d(finalVx, finalVy, Math.toRadians(finalVo)))
                                .lineToLinearHeading(new Pose2d(finalVx, ywallstart, Math.toRadians(owallstart)))
                                .lineToLinearHeading(new Pose2d(xwallstart, ywallstart, Math.toRadians(owallstart)))
                                .build()


                        /*.forward(30)
                                .turn(Math.toRadians(90))
                                .forward(30)
                                .turn(Math.toRadians(90))
                                .forward(30)
                                .turn(Math.toRadians(90))
                                .forward(30)
                                .turn(Math.toRadians(90))
                                .build()*/

                );

        // Declare out second bot


        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)

                // Add both of our declared bot entities
                .addEntity(myFirstBot)

                .start();
    }
}