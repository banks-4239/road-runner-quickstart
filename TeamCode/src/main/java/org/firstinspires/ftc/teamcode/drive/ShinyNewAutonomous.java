package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.onbotjava.handlers.objbuild.WaitForBuild;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.RobotReference;

import java.util.List;

import kotlin.random.Random;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")

public class ShinyNewAutonomous extends LinearOpMode {

    RobotReference rb = new RobotReference();
    
    //Blue Poses
    Vector2d vectorDuckSpinBlue = new Vector2d(-60,56.7);
    Pose2d duckSpinBlue = new Pose2d(-60,56.7, Math.toRadians(90));
    Pose2d blueStorageUnit = new Pose2d(-63, 37, Math.toRadians(0));
    Pose2d freightBlueDuck = new Pose2d(-33, 24, Math.toRadians(0));
    Pose2d freightBlueWarehouse = new Pose2d(-12, 45, Math.toRadians(-90));
    Pose2d startPosBlueDuck = new Pose2d(-41, 63.5, Math.toRadians(90));
    Pose2d startPosBlueWarehouse = new Pose2d(7, 63.5, Math.toRadians(90));
    Pose2d endPosBlueWarehouse = new Pose2d(65.4, 36, Math.toRadians(-90));
    Pose2d blueIntermediate1 = new Pose2d(-33, 65.4, Math.toRadians(0));
    Pose2d blueIntermediate2 = new Pose2d(7, 65.4, Math.toRadians(0));
    Pose2d blueIntermediate3 = new Pose2d(38, 65.4, Math.toRadians(0));
    Pose2d blueIntermediate4 = new Pose2d(38, 45, Math.toRadians(0));
    Pose2d blueIntermediateDuck = new Pose2d(-58, 16, Math.toRadians(45));
    Vector2d vectorBlueIntermediateDuck = new Vector2d(-58, -16);

    //Red Poses

    Vector2d vectorDuckSpinRed = new Vector2d(-60,-56.7);
    Pose2d duckSpinRed = new Pose2d(vectorDuckSpinRed, Math.toRadians(-90));
    Pose2d redStorageUnit = new Pose2d(-63, -37, Math.toRadians(0));
    Pose2d freightRedDuck = new Pose2d(-33, -24, Math.toRadians(0));
    Pose2d freightRedWarehouse = new Pose2d(-12, -45, Math.toRadians(90));
    Pose2d startPosRedDuck = new Pose2d(-41,-63.5, Math.toRadians(-90));
    Pose2d startPosRedWarehouse = new Pose2d(7, -63.5, Math.toRadians(-90));
    Pose2d endPosRedWarehouse = new Pose2d(65.4, -36, Math.toRadians(90));
    Pose2d redIntermediate1 = new Pose2d(-33, -65.4, Math.toRadians(0));
    Pose2d redIntermediate2 = new Pose2d(7, -65.4, Math.toRadians(0));
    Pose2d redIntermediate3 = new Pose2d(38, -65.4, Math.toRadians(0));
    Pose2d redIntermediate4 = new Pose2d(38, -45, Math.toRadians(0));
    Pose2d redIntermediateDuck = new Pose2d(-58, -16, Math.toRadians(-45));
    Vector2d vectorRedIntermediateDuck = new Vector2d(-58, -16);

    public int hubNum;
    public int test;

    @Override
    public void runOpMode() throws InterruptedException {

        rb.init(hardwareMap);

        initVuforia();
        initTfod();
        if (rb.tfod != null) {
            rb.tfod.activate();

            rb.tfod.setZoom(1, 16.0/9.0);
        }
        telemetry.addData("Please choose a mode!", "up - redDuck, right - redWarehouse, left - blueDuck, down - blueWarehouse");
        telemetry.update();

        while (!rb.choosingAuto) {


            telemetry.addData("Please choose a setting", "");
            if (rb.redOrBlue) {
                telemetry.addData("Red", rb.settings[rb.onSetting - 1]);
                telemetry.addData("Red", ">" + rb.settings[rb.onSetting]);
                telemetry.addData("Red", rb.settings[rb.onSetting + 1]);
            } else {
                telemetry.addData("Blue", rb.settings[rb.onSetting - 1]);
                telemetry.addData("Blue", ">" + rb.settings[rb.onSetting]);
                telemetry.addData("Blue", rb.settings[rb.onSetting + 1]);
            }

            telemetry.addData("hub", hubNum);

            if (gamepad1.dpad_up) {
                if (rb.onSetting > 1 && !rb.settingButtonDown1) {
                    rb.onSetting--;
                }
                rb.settingButtonDown1 = true;
            } else {
                rb.settingButtonDown1 = false;
            }

            if (gamepad1.dpad_down) {
                if (rb.onSetting < 3 && !rb.settingButtonDown2) {
                    rb.onSetting++;
                }
                rb.settingButtonDown2 = true;
            } else {
                rb.settingButtonDown2 = false;
            }

            if (gamepad1.dpad_left) {
                rb.redOrBlue = true;
            } else if (gamepad1.dpad_right) {
                rb.redOrBlue = false;
            }

            if (isStarted()) {
                if (rb.redOrBlue) {
                    rb.choosingAuto = true;
                    rb.autoMode = rb.onSetting;
                } else {
                    rb.choosingAuto = true;
                    rb.autoMode = rb.onSetting + 3;
                }
            }
            hubNum = getElement();
            telemetry.update();


        }
        waitForStart();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        switch (rb.autoMode) {
            case 1:
                redDuckToStorageUnit();
                break;
            case 2:
                redDuckToWarehouse();
                break;
            case 3:
                redWarehouseWithFreight();
                break;
            case 4:
                blueDuckToStorageUnit();
                //flipHub();
            case 5:
                blueDuckToWarehouse();
                //flipHub();
                break;
            case 6:
                blueWarehouseWithFreight();
                //flipHub();
                break;

        }

    }


    public void redDuckToStorageUnit(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Trajectory red1 = drive.trajectoryBuilder(startPosRedDuck)
                .lineToLinearHeading(duckSpinRed)
                .build();

        Trajectory red2 = drive.trajectoryBuilder(red1.end())
                .lineToLinearHeading(redIntermediateDuck)
                .build();

        Trajectory red3 = switchTrajRedDuck(red2.end());

        Trajectory red4 = drive.trajectoryBuilder(red3.end())
                .lineToLinearHeading(redIntermediateDuck)
                .build();

        Trajectory red5 = drive.trajectoryBuilder(red4.end())
                .lineToLinearHeading(redStorageUnit)
                .build();


        waitForStart();

        if (isStopRequested()) return;
        drive.setPoseEstimate(startPosRedDuck);
        drive.followTrajectory(red1);
        spinnerRed(0.3);
        sleep(4000);
        spinnerEnd();
        drive.followTrajectory(red2);
        drive.followTrajectory(red3);

        switch(hubNum){
            default:
                //scoring
                liftArm(rb.LIFT_2,rb.LIFT_ARM_ROTATE_PWR);
                waitForArm(rb.LIFT_2);
                takeOut(1);
                sleep(500);
                intakeOff();
                liftArm(rb.LIFT_0,rb.LIFT_ARM_ROTATE_PWR);
                waitForArm(0);
                break;
            case 2:
                //scoring
                liftArm(rb.LIFT_6,rb.LIFT_ARM_ROTATE_PWR);
                waitForArm(rb.LIFT_6);
                takeOut(1);
                sleep(500);
                intakeOff();
                liftArm(rb.LIFT_0,rb.LIFT_ARM_ROTATE_PWR);
                waitForArm(0);
                break;
            case 3:
                //scoring
                liftArm(rb.LIFT_5,rb.LIFT_ARM_ROTATE_PWR);
                waitForArm(rb.LIFT_5);
                takeOut(1);
                sleep(500);
                intakeOff();
                liftArm(rb.LIFT_0,rb.LIFT_ARM_ROTATE_PWR);
                waitForArm(0);
                break;
        }

        sleep(5200);
        spinnerEnd();
        drive.followTrajectory(red4);
        drive.followTrajectory(red5);
    }
    public void redDuckToWarehouse(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Trajectory red1 = drive.trajectoryBuilder(startPosRedDuck)
                .lineToLinearHeading(duckSpinRed)
                .build();
        Trajectory red2 = drive.trajectoryBuilder(red1.end())
                .lineToLinearHeading(redIntermediateDuck)
                .build();
        Trajectory red3 = (switchTrajRedDuck(red2.end()));

        Trajectory red4 = drive.trajectoryBuilder(red3.end())
                .lineTo(vectorRedIntermediateDuck)
                .build();
        Trajectory red5 = drive.trajectoryBuilder(red4.end())
                .lineToLinearHeading(new Pose2d(duckSpinRed.getX(), duckSpinRed.getY() + 5, Math.toRadians(0)))
                .build();
        Trajectory red6 = drive.trajectoryBuilder(red5.end())
                .lineToLinearHeading(new Pose2d(redIntermediate2.getX(), redIntermediate2.getY() - 0.5, redIntermediate2.getHeading()))
                .build();
        Trajectory red7 = drive.trajectoryBuilder(red6.end())
                .lineToLinearHeading(new Pose2d(redIntermediate3.getX(), redIntermediate3.getY() - 0.5, redIntermediate3.getHeading()))
                .build();
        Trajectory red8 = drive.trajectoryBuilder(red7.end())
                .lineToLinearHeading(redIntermediate4)
                .build();
        //Trajectory red9 = drive.trajectoryBuilder(red8.end())
        //        .lineToLinearHeading(endPosRedWarehouse)
        //        .build();
        waitForStart();
        if (isStopRequested()) return;
        drive.setPoseEstimate(startPosRedDuck);
        drive.followTrajectory(red1);
        spinnerRed(0.3);
        sleep(4000);
        spinnerEnd();
        drive.followTrajectory(red2);
        drive.followTrajectory(red3);

        switch(hubNum){
            default:
                //scoring
                liftArm(rb.LIFT_2,rb.LIFT_ARM_ROTATE_PWR);
                waitForArm(rb.LIFT_2);
                takeOut(1);
                sleep(500);
                intakeOff();
                liftArm(rb.LIFT_0,rb.LIFT_ARM_ROTATE_PWR);
                waitForArm(0);
                break;
            case 2:
                //scoring
                liftArm(rb.LIFT_6,rb.LIFT_ARM_ROTATE_PWR);
                waitForArm(rb.LIFT_6);
                takeOut(1);
                sleep(500);
                intakeOff();
                liftArm(rb.LIFT_0,rb.LIFT_ARM_ROTATE_PWR);
                waitForArm(0);
                break;
            case 3:
                //scoring
                liftArm(rb.LIFT_5,rb.LIFT_ARM_ROTATE_PWR);
                waitForArm(rb.LIFT_5);
                takeOut(1);
                sleep(500);
                intakeOff();
                liftArm(rb.LIFT_0,rb.LIFT_ARM_ROTATE_PWR);
                waitForArm(0);
                break;
        }

        drive.followTrajectory(red4);
        drive.followTrajectory(red5);
        drive.followTrajectory(red6);
        drive.followTrajectory(red7);
        drive.followTrajectory(red8);

    }
    public void redWarehouseWithFreight(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);



        Trajectory red1 = drive.trajectoryBuilder(startPosRedWarehouse)
                .lineToLinearHeading(freightRedWarehouse)
                .build();

        Trajectory red2 = drive.trajectoryBuilder(red1.end())
                .lineToLinearHeading(redIntermediate2)
                .build();

        Trajectory red3 = drive.trajectoryBuilder(red2.end())
                .lineToLinearHeading(redIntermediate3)
                .build();

        Trajectory red4 = drive.trajectoryBuilder(red3.end())
                .lineToLinearHeading(redIntermediate4)
                .build();

        //Trajectory red5 = drive.trajectoryBuilder(red4.end())
        //        .lineToLinearHeading(endPosRedWarehouse)
        //        .build();


        waitForStart();

        if (isStopRequested()) return;
        drive.setPoseEstimate(startPosRedWarehouse);
        drive.followTrajectory(red1);
        drive.followTrajectory(red2);
        drive.followTrajectory(red3);
        drive.followTrajectory(red4);
        //drive.followTrajectory(red5);
    }
    public void blueDuckToStorageUnit(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Trajectory blue1 = drive.trajectoryBuilder(startPosBlueDuck)
                .lineToLinearHeading(duckSpinBlue)
                .build();

        Trajectory blue2 = drive.trajectoryBuilder(blue1.end())
                .lineToLinearHeading(freightBlueDuck)
                .build();

        Trajectory blue3 = drive.trajectoryBuilder(blue2.end())
                .lineToLinearHeading(blueStorageUnit)
                .build();


        waitForStart();

        if (isStopRequested()) return;
        drive.setPoseEstimate(startPosRedDuck);
        drive.followTrajectory(blue1);
        spinnerBlue(0.3);
        sleep(5200);
        spinnerEnd();
        drive.followTrajectory(blue2);
        drive.followTrajectory(blue3);

    }
    public void blueDuckToWarehouse(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Trajectory blue1 = drive.trajectoryBuilder(startPosBlueDuck)
                .lineToLinearHeading(duckSpinBlue)
                .build();
        Trajectory blue2 = drive.trajectoryBuilder(blue1.end())
                .lineToLinearHeading(blueIntermediateDuck)
                .build();
        Trajectory blue3 = drive.trajectoryBuilder(blue2.end())
                .lineToLinearHeading(freightRedDuck)
                .build();
        Trajectory blue4 = drive.trajectoryBuilder(blue3.end())
                .lineTo(vectorBlueIntermediateDuck)
                .build();
        Trajectory blue5 = drive.trajectoryBuilder(blue4.end())
                .lineTo(vectorDuckSpinBlue)
                .build();
        Trajectory blue6 = drive.trajectoryBuilder(blue5.end())
                .lineToLinearHeading(new Pose2d(blueIntermediate2.getX(), blueIntermediate2.getY() - 0.5, blueIntermediate2.getHeading()))
                .build();
        Trajectory blue7 = drive.trajectoryBuilder(blue6.end())
                .lineToLinearHeading(new Pose2d(redIntermediate3.getX(), redIntermediate3.getY() - 0.5, redIntermediate3.getHeading()))
                .build();
        Trajectory blue8 = drive.trajectoryBuilder(blue7.end())
                .lineToLinearHeading(blueIntermediate4)
                .build();
        //Trajectory blue9 = drive.trajectoryBuilder(blue8.end())
        //        .lineToLinearHeading(endPosBlueWarehouse)
        //        .build();
        waitForStart();
        if (isStopRequested()) return;
        drive.setPoseEstimate(startPosBlueDuck);
        drive.followTrajectory(blue1);
        spinnerBlue(0.3);
        sleep(5200);
        spinnerEnd();
        drive.followTrajectory(blue2);
        drive.followTrajectory(blue3);
        drive.followTrajectory(blue4);
        drive.followTrajectory(blue5);
        drive.followTrajectory(blue6);
        drive.followTrajectory(blue7);
        drive.followTrajectory(blue8);
        //drive.followTrajectory(blue9);

    }
    public void blueWarehouseWithFreight(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Trajectory blue1 = drive.trajectoryBuilder(startPosBlueWarehouse)
                .lineToLinearHeading(freightBlueWarehouse)
                .build();

        Trajectory blue2 = drive.trajectoryBuilder(blue1.end())
                .lineToLinearHeading(blueIntermediate2)
                .build();

        Trajectory blue3 = drive.trajectoryBuilder(blue2.end())
                .lineToLinearHeading(blueIntermediate3)
                .build();

        Trajectory blue4 = drive.trajectoryBuilder(blue3.end())
                .lineToLinearHeading(blueIntermediate4)
                .build();

        //Trajectory blue5 = drive.trajectoryBuilder(blue4.end())
        //        .lineToLinearHeading(endPosBlueWarehouse)
        //        .build();

        waitForStart();
        
        if (isStopRequested()) return;
        drive.setPoseEstimate(startPosBlueWarehouse);
        drive.followTrajectory(blue1);
        drive.followTrajectory(blue2);
        drive.followTrajectory(blue3);
        drive.followTrajectory(blue4);
        //drive.followTrajectory(blue5);
    }

    public Trajectory switchTrajRedDuck(Pose2d pose){

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        switch (hubNum){
            case 2:
                return drive.trajectoryBuilder(pose)
                        .lineToLinearHeading(new Pose2d(freightRedDuck.getX() - 2, freightRedDuck.getY(), freightRedDuck.getHeading() + Math.toRadians(0)))
                        .build();
            case 3:
                return drive.trajectoryBuilder(pose)
                        .lineToLinearHeading(new Pose2d(freightRedDuck.getX(), freightRedDuck.getY(), freightRedDuck.getHeading() + Math.toRadians(0)))
                        .build();
            default:
                return drive.trajectoryBuilder(pose)
                        .lineToLinearHeading(new Pose2d(freightRedDuck.getX() - 1, freightRedDuck.getY(), freightRedDuck.getHeading() + Math.toRadians(180)))
                        .build();
        }
    }





    public void spinnerRed(double speed) {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.spinnerR.setPower(speed);
        drive.spinnerL.setPower(speed);
    }
    public void spinnerBlue(double speed) {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.spinnerR.setPower(-speed);
        drive.spinnerL.setPower(-speed);
    }
    public void spinnerEnd() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.spinnerR.setPower(0);
        drive.spinnerL.setPower(0);
    }

    public void liftArm(int ticks, double power) {
        rb.robotArm.setTargetPosition(ticks);
        rb.robotArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.robotArm.setPower(power);
    }

    public void waitForArm(int ticks) {
        while (!(rb.robotArm.getCurrentPosition() >= ticks - 100) || !(rb.robotArm.getCurrentPosition() <= ticks + 100))
        {
            if(rb.robotArm.getCurrentPosition() >= test)
            {
                test = rb.robotArm.getCurrentPosition();
                telemetry.addData("", test);
            }
            telemetry.update();
        }


    }

    public void takeOut(double speed) {
        rb.intake.setPower(speed);
    }

    void intakeOff() {
        rb.intake.setPower(0);
    }



    public int getElement(){

        if (rb.tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = rb.tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                // telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (i = 0; i != updatedRecognitions.size(); i++) {
                        /*telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());*/

                    if(updatedRecognitions.get(i).getLabel() != "Marker") {
                        telemetry.addData("", updatedRecognitions.get(0).getLeft());
                        if(updatedRecognitions.get(0).getLeft() < 180){
                            return 1;
                        }else{
                            return 2;
                        }
                    }






                }
                if(updatedRecognitions.size() == 0){
                    return 3;
                }





            }

        }
        return hubNum;
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = rb.VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        rb.vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        rb.tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, rb.vuforia);
        rb.tfod.loadModelFromAsset(rb.TFOD_MODEL_ASSET, rb.LABELS);
    }






}
/*public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory[] trajarray = new Trajectory[2];



        trajarray[0] = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(10)
                .build();

        trajarray[1]  = drive.trajectoryBuilder(trajarray[0].end())
                .forward(5)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        for(int i = 0; i == trajarray.length; i++){
            drive.followTrajectory(trajarray[i]);
        }

    }*/