package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class meepmeep {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d duckSpinRed = new Pose2d(-60, -56.5, Math.toRadians(-90));
        Pose2d redStorageUnit = new Pose2d(-63, -37, Math.toRadians(0));
        Pose2d freightRedDuck = new Pose2d(-33, -24, Math.toRadians(0));
        Pose2d freightRedWarehouse = new Pose2d(-12, -45, Math.toRadians(90));
        Pose2d startPosRedDuck = new Pose2d(-41, -63.5, Math.toRadians(-90));
        Pose2d startPosRedWarehouse = new Pose2d(7, -63.5, Math.toRadians(-90));
        Pose2d endPosRedWarehouse = new Pose2d(65.4, -36, Math.toRadians(90));
        Pose2d redIntermediate1 = new Pose2d(-33, -65.4, Math.toRadians(0));
        Pose2d redIntermediate2 = new Pose2d(7, -65.4, Math.toRadians(0));
        Pose2d redIntermediate3 = new Pose2d(38, -65.4, Math.toRadians(0));
        Pose2d redIntermediate4 = new Pose2d(38, -38, Math.toRadians(0));
        Pose2d redIntermediateDuck = new Pose2d(-58, -16, Math.toRadians(-90));
        // Declare our first bot
        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue

                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPosRedDuck)
                                .lineToLinearHeading(duckSpinRed)
                                .lineToLinearHeading(redIntermediateDuck)
                                .lineToLinearHeading(freightRedDuck)
                                .lineToLinearHeading(redIntermediateDuck)
                                .lineToLinearHeading(duckSpinRed)
                                .lineToLinearHeading(redIntermediate1)
                                .lineToLinearHeading(redIntermediate3)
                                .lineToLinearHeading(redIntermediate4)
                                .build()
                );

        // Declare out second bot
        /*
        RoadRunnerBotEntity mySecondBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(30, 30, Math.toRadians(180)))
                                .forward(30)
                                .turn(Math.toRadians(90))
                                .forward(30)
                                .turn(Math.toRadians(90))
                                .forward(30)
                                .turn(Math.toRadians(90))
                                .forward(30)
                                .turn(Math.toRadians(90))
                                .build()
                );
*/
        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)

                // Add both of our declared bot entities
                .addEntity(myFirstBot)
                // .addEntity(mySecondBot)
                .start();
    }
}