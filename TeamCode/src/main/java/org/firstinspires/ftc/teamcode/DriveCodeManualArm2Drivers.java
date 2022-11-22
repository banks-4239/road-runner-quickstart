package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp(group = "advanced")

public class DriveCodeManualArm2Drivers extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Servo clawServo = hardwareMap.get(Servo.class, "claw");
        DcMotor armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // this is what we get from autonomous
        drive.setPoseEstimate(PoseStorage.currentPose);

        boolean clawOpen = false;
        boolean yPressed = false;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            //drive.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            if(gamepad2.dpad_up){
                //move arm up
                armMotor.setPower(-.5);
                clawServo.setPosition(1);
                clawOpen = false;
            }else if(gamepad2.dpad_down){
                //move arm down
                armMotor.setPower(.5);
                clawServo.setPosition(1);
                clawOpen = false;
            }else{
                armMotor.setPower(0);
            }

            if(gamepad2.y && clawOpen && !yPressed){
                //close claw
                clawServo.setPosition(1);
                clawOpen = false;
                yPressed = true;
            }
            if (gamepad2.y && !clawOpen && !yPressed) {
                //Open claw
                clawServo.setPosition(.9);
                clawOpen = true;
                yPressed = true;
            }
            if (!gamepad2.y && yPressed) {
                yPressed = false;
            }

            drive.setWeightedDrivePower(
                    new Pose2d(
                            //-forward/backward
                            -(gamepad1.left_stick_y/2),
                            //-strafe
                            -(gamepad1.left_stick_x/2),
                            //-rotate
                            -(gamepad1.right_stick_x/2)
                    )
            );

            // Update everything. Odometry. Etc.
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Print pose to telemetry
            //telemetry.addData("x", poseEstimate.getX());
            //telemetry.addData("y", poseEstimate.getY());
            //telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("Claw is open = ", clawOpen);
            telemetry.addData("Arm Height", armMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}