package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.onbotjava.handlers.objbuild.WaitForBuild;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.RobotReference;

import java.util.List;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")

public class ShinyNewAutonomous extends LinearOpMode {

    RobotReference rb = new RobotReference();

    
    
    //static Pose2d START_POSE = new Pose2d(-41, -62, Math.toRadians(90));

    //Blue Poses
    Pose2d duckSpinBlue = new Pose2d(-60,56.1, Math.toRadians(90));
    Pose2d blueStorageUnit = new Pose2d(-63, 37, Math.toRadians(0));
    Pose2d freightBlueDuck = new Pose2d(-33, 24, Math.toRadians(0));
    Pose2d freightBlueWarehouse = new Pose2d(-12, 45, Math.toRadians(-90));
    Pose2d startPosBlueDuck = new Pose2d(-41, 63, Math.toRadians(90));
    Pose2d startPosBlueWarehouse = new Pose2d(7, 63, Math.toRadians(90));
    Pose2d endPosBlueWarehouse = new Pose2d(65, 38, Math.toRadians(-90));
    Pose2d blueIntermediate1 = new Pose2d(-33, 65, Math.toRadians(0));
    Pose2d blueIntermediate2 = new Pose2d(7, 65, Math.toRadians(0));
    Pose2d blueIntermediate3 = new Pose2d(38, 65, Math.toRadians(0));
    Pose2d blueIntermediate4 = new Pose2d(38, 38, Math.toRadians(0));

    //Red Poses
    Pose2d duckSpinRed = new Pose2d(-60,-56.1, Math.toRadians(-90));
    Pose2d redStorageUnit = new Pose2d(-63, -37, Math.toRadians(0));
    Pose2d freightRedDuck = new Pose2d(-33, -24, Math.toRadians(0));
    Pose2d freightRedWarehouse = new Pose2d(-12, -45, Math.toRadians(90));
    Pose2d startPosRedDuck = new Pose2d(-41,63, Math.toRadians(-90));
    Pose2d startPosRedWarehouse = new Pose2d(7, -63, Math.toRadians(-90));
    Pose2d endPosRedWarehouse = new Pose2d(65, -38, Math.toRadians(90));
    Pose2d redIntermediate1 = new Pose2d(-33, -65, Math.toRadians(0));
    Pose2d redIntermediate2 = new Pose2d(7, -65, Math.toRadians(0));
    Pose2d redIntermediate3 = new Pose2d(38, -65, Math.toRadians(0));
    Pose2d redIntermediate4 = new Pose2d(38, -38, Math.toRadians(0));

