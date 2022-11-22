package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/**
 * Example opmode demonstrating how to hand-off the pose from your autonomous opmode to your teleop
 * by passing the data through a static class.
 * <p>
 * This is required if you wish to read the pose from odometry in teleop and you run an autonomous
 * sequence prior. Without passing the data between each other, teleop isn't completely sure where
 * it starts.
 * <p>
 * This example runs the same paths used in the SplineTest tuning opmode. After the trajectory
 * following concludes, it simply sets the static value, `PoseStorage.currentPose`, to the latest
 * localizer reading.
 * However, this method is not foolproof. The most immediate problem is that the pose will not be
 * written to the static field if the opmode is stopped prematurely. To work around this issue, you
 * need to continually write the pose to the static field in an async trajectory follower. A simple
 * example of async trajectory following can be found at
 * https://www.learnroadrunner.com/advanced.html#async-following
 * A more advanced example of async following can be found in the AsyncFollowingFSM.java class.
 * <p>
 * The other edge-case issue you may want to cover is saving the pose value to disk by writing it
 * to a file in the event of an app crash. This way, the pose can be retrieved and set even if
 * something disastrous occurs. Such a sample has not been included.
 */
@Autonomous

public class OurAutonomous extends LinearOpMode {

    int xReflect;
    int rotateReflect;
    int armPositionHighScore = -2669;
    int armPositionMidScore = -2086;
    int armPositionLowScore = -1415;
    int armPositionStartingLocation = 0;
    int armPositionConeStack = -635;
    double armMotorPower = 0.5;
    int armPositionLiftConeStack = -550;
    int armPositionConeStackDifference = 125;
    double clawOffset = 1.5;
    double tileWidth = 23.5;
    double speedConstant = 0.5;
    double slow = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare your drive class
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        Servo clawServo = hardwareMap.get(Servo.class, "claw");
        DcMotor armMotor = hardwareMap.get(DcMotorEx.class, "arm");

        while(!gamepad1.a){
            if(gamepad1.dpad_left){
                //LEFT side
                xReflect = 1;
                rotateReflect = 0;
                telemetry.addData("Side: ","left");
            }else if(gamepad1.dpad_right){
                //Right side
                xReflect = -1;
                rotateReflect = 180;
                telemetry.addData("Side: ","right");
            }
            telemetry.update();
        }

        // Set the pose estimate to where you know the bot will start in autonomous
        // Refer to https://www.learnroadrunner.com/trajectories.html#coordinate-system for a map
        // of the field
        // This example sets the bot at x: 10, y: 15, and facing 90 degrees (turned counter-clockwise)
        Pose2d startPose = new Pose2d(-35.25* xReflect, -62
                , Math.toRadians(90));
//        Vector2d redLeft1 = new Vector2d(-0.5*tileWidth, -2.5*tileWidth);
//        Vector2d redLeft2 = new Vector2d(-0.5*tileWidth, -1.5*tileWidth);
//        //rotate-90
//        Vector2d redLeftScoreSetup = new Vector2d(-0.5*tileWidth, -1*tileWidth+clawOffset);
//        Vector2d redLeftScore = new Vector2d(-6.75, -25.5+clawOffset);
//        //redleftscoresetup
//        Vector2d redLeft3 = new Vector2d(-0.5*tileWidth, -0.5*tileWidth);
//        //rotate180
//        Vector2d redLeftConeStackSetup = new Vector2d(-54, -11.75-clawOffset);
//        Vector2d redLeftConeStack = new Vector2d(-60.5, -11.75-clawOffset);
        Pose2d redLeft1 = new Pose2d(-11.75* xReflect, -58.75, Math.toRadians(90));
        Pose2d redLeft2 = new Pose2d(-11.75* xReflect, -35.25, Math.toRadians(90));

        //rotate-90
        Pose2d redLeftScoreSetup = new Pose2d(-11.75* xReflect, -22.5 + clawOffset, Math.toRadians(rotateReflect-0));
        Pose2d redLeftScore = new Pose2d((-11.75 + 3.5)* xReflect, -22.5 + clawOffset, Math.toRadians(rotateReflect-0));
        //redleftscoresetup Original y value for above poses: -24.5
        Pose2d redLeft3 = new Pose2d(-11.75* xReflect, -5.75,Math.toRadians(rotateReflect-0));
//        Pose2d redLeft3 = new Pose2d(RedLeft3, Math.toRadians(0));
//        Pose2d redLeft4 = new Pose2d(RedLeft3, Math.toRadians(180));
        //rotate180 Original y value for above and below poses: -11.75
        Pose2d redLeftConeStackSetup = new Pose2d(-55* xReflect, -5.75, Math.toRadians(rotateReflect-180));
        Pose2d redLeftConeStack = new Pose2d(-64* xReflect, -5.75, Math.toRadians(rotateReflect-180));
        Pose2d redLeft4 = new Pose2d((-11.75-3.25)* xReflect, -5.75,Math.toRadians(rotateReflect-180));
        Pose2d redLeftScoreSetup2 = new Pose2d((-11.75-3.25)* xReflect, -22.5 + clawOffset, Math.toRadians(rotateReflect-0));
        Pose2d redLeftScore2 = new Pose2d((-11.75 + 3.5 -3.25)* xReflect, -22.5 + clawOffset, Math.toRadians(rotateReflect-0));
        //redleftconestacksetup
        //redLeft3
        //redLeftScoreSetup

        robot.setPoseEstimate(startPose);


        TrajectorySequence redLeftStartToHighPole = robot.trajectorySequenceBuilder(startPose)
                //strafe to right
                .lineToLinearHeading(redLeft1, SampleMecanumDrive.getVelocityConstraint((DriveConstants.MAX_VEL * speedConstant), DriveConstants.MAX_ANG_VEL * speedConstant, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * speedConstant))

                .addDisplacementMarker(() -> {
                    robot.moveArmTo(armPositionHighScore);
                })
                //move forward by 1 square
                .lineToLinearHeading(redLeft2, SampleMecanumDrive.getVelocityConstraint((DriveConstants.MAX_VEL * speedConstant), DriveConstants.MAX_ANG_VEL * speedConstant, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * speedConstant))

                //turns to the right
                .turn(Math.toRadians(-90* xReflect))
                //strafe left to pole (high)
                .lineToLinearHeading(redLeftScoreSetup, SampleMecanumDrive.getVelocityConstraint((DriveConstants.MAX_VEL * speedConstant), DriveConstants.MAX_ANG_VEL * speedConstant, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * speedConstant))
                .build();
        Trajectory redLeftMoveToScore = robot.trajectoryBuilder(redLeftStartToHighPole.end())
                .forward(4.5)
//                .lineToLinearHeading(redLeftScore, SampleMecanumDrive.getVelocityConstraint((DriveConstants.MAX_VEL * speedConstant), DriveConstants.MAX_ANG_VEL * speedConstant, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * speedConstant))
                .build();
        Trajectory redLeftBackOffOfPole = robot.trajectoryBuilder(redLeftMoveToScore.end())
                .lineToLinearHeading(redLeftScoreSetup, SampleMecanumDrive.getVelocityConstraint((DriveConstants.MAX_VEL * speedConstant), DriveConstants.MAX_ANG_VEL * speedConstant, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * speedConstant))
                .build();
        TrajectorySequence redLeftGoToConeStack = robot.trajectorySequenceBuilder(redLeftBackOffOfPole.end())
                .lineToLinearHeading(redLeft3, SampleMecanumDrive.getVelocityConstraint((DriveConstants.MAX_VEL * speedConstant), DriveConstants.MAX_ANG_VEL * speedConstant, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * speedConstant))
                .turn(Math.toRadians(180*xReflect))
                .lineToLinearHeading(redLeftConeStackSetup, SampleMecanumDrive.getVelocityConstraint((DriveConstants.MAX_VEL * speedConstant), DriveConstants.MAX_ANG_VEL * speedConstant, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * speedConstant))
                .addDisplacementMarker(() -> {
                    robot.openClaw();
                })
                .lineToLinearHeading(redLeftConeStack, SampleMecanumDrive.getVelocityConstraint((DriveConstants.MAX_VEL / 5), DriveConstants.MAX_ANG_VEL * speedConstant, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * speedConstant))
                .build();
        Trajectory redLeftBackOffOfConeStack = robot.trajectoryBuilder(redLeftGoToConeStack.end())
                .lineToLinearHeading(redLeftConeStackSetup, SampleMecanumDrive.getVelocityConstraint((DriveConstants.MAX_VEL), DriveConstants.MAX_ANG_VEL * speedConstant, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * speedConstant))
                .build();
        TrajectorySequence redLeftConeStackSetupToHighPoleSetup = robot.trajectorySequenceBuilder(redLeftBackOffOfConeStack.end())
                .lineToLinearHeading(redLeft4, SampleMecanumDrive.getVelocityConstraint((DriveConstants.MAX_VEL * speedConstant), DriveConstants.MAX_ANG_VEL * speedConstant, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * slow))
                .turn(Math.toRadians(180*xReflect))
                .lineToLinearHeading(redLeftScoreSetup2, SampleMecanumDrive.getVelocityConstraint((DriveConstants.MAX_VEL * speedConstant), DriveConstants.MAX_ANG_VEL * speedConstant, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * speedConstant))
                .build();

        //    TrajectorySequence name = robot.trajectorySequenceBuilder(redLeftStartToHighPole.end())
        //            .build();

        //    Trajectory name = robot.trajectoryBuilder(redLeftStartToHighPole.end())
        //            .build();


        robot.closeClaw();
        waitForStart();


        robot.followTrajectorySequence(redLeftStartToHighPole);

        for (int i = 0; i < 5; i++) {
            robot.followTrajectory(redLeftMoveToScore);
            sleep(250);
            robot.openClaw();
            sleep(250);
            robot.followTrajectory(redLeftBackOffOfPole);
            robot.moveArmTo(armPositionConeStack + armPositionConeStackDifference * i);
            robot.followTrajectorySequence(redLeftGoToConeStack);
            robot.closeClaw();
            sleep(250);
            //liftConeFromStack
            robot.moveArmTo(armPositionConeStack + armPositionConeStackDifference * i + armPositionLiftConeStack);
            sleep(1000);

            robot.followTrajectory(redLeftBackOffOfConeStack);
            robot.moveArmTo(armPositionHighScore);
            robot.followTrajectorySequence(redLeftConeStackSetupToHighPoleSetup);
        }

        // Transfer the current pose to PoseStorage so we can use it in TeleOp
        PoseStorage.currentPose = robot.getPoseEstimate();
    }

}