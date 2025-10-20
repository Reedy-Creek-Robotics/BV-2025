package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Arrays;
import java.util.List;
import java.util.Objects;

@Autonomous(name = "Blue Auto with camera")
public class Auto extends LinearOpMode{
    private final Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);
    DcMotor driveFrontLeft, driveFrontRight, driveBackRight, driveBackLeft, outtake, leftIntake, rightIntake;
    Servo outtakeHammer, transfer;
    IMU imu;
    AprilTagProcessor aprilTag;
    private BuiltinCameraDirection webcamName ;
    private VisionPortal visionportal;

    public static String MOTIFPATTERN;

    //PP
    private Follower follower;
    static Pose endPose;

    private Pose initPos;
    public final Pose scorePose = new Pose(60, 85, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(19.000, 84.000, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(19.000, 60, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(19.000, 36, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private Supplier<PathChain> grabPickup1,  grabPickup2;

    // possible states for Auto
    int autoState = 1,
        intakeState = 1;

    String artifactOrder = "PPG";

    ElapsedTime scoreDelay = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS),
               intakeDelay = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    double
            intake1 = (double) 0 /300,
            intake2 = (double) 120 /300,
            intake3 = (double) 240 /300,
            outtake2 = (double) 30/300,
            outtake1 = (double) 150 /300,
            outtake3 = (double) 270 /300;
    private boolean scoring = false,
            intaking = false;

    int scoringState = 1;


    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        setApriltag();
        List results= AprilTagDetection();
        initPos = (Pose) results.get(1);
        MOTIFPATTERN = results.get(0).toString();
        visionportal.close();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(initPos);
        telemetry.addData("MOTIF",MOTIFPATTERN);
        buildPaths();
        waitForStart();
        transfer.setPosition(0);
        while(opModeIsActive()){
            updateIntake();
            updateScore();
            follower.update();
            switch (autoState){
                case 1:
                    scoring = true;
                    autoState++;
                    telemetry.addData("Current action","scoring");
                case 2:
                    if(!scoring){
                        artifactOrder = "PPG";
                        follower.followPath(grabPickup1.get());
                        autoState++;
                        telemetry.addData("Current action","grabbing artifacts");
                    }
                    break;
                case 3:
                    if(!follower.isBusy()){
                        scoring = true;
                        autoState++;
                        telemetry.addData("Current action","scoring");
                    }
                    break;
                case 4:
                    artifactOrder = "PGP";
                    if(!follower.isBusy() && !scoring) {
                        follower.followPath(grabPickup2.get());
                        autoState++;
                        telemetry.addData("Current action","grabbing artifacts");
                    }
                    break;
                case 6:
                    if(!follower.isBusy()){
                        scoring = true;
                        autoState=-1;
                        telemetry.addData("Current action","scoring");
                    }
                    break;

            }
        endPose=follower.getPose();
        }
    }

    private void updateScore() {
        if(scoring){
           if(Objects.equals(MOTIFPATTERN, "PPG")){
               if(Objects.equals(artifactOrder, "PPG")){
                   switch (scoringState){
                       case 1:
                           outtake.setPower(1);
                           transfer.setPosition(outtake1);
                           scoreDelay.reset();
                           scoringState++;
                           break;
                       case 2:
                       case 5:
                       case 8:
                           if(scoreDelay.milliseconds()>50){
                               outtakeHammer.setPosition((double) 110 /300);
                               scoreDelay.reset();
                               scoringState++;
                           }
                           break;
                       case 3:
                       case 6:
                       case 9:
                           if(scoreDelay.milliseconds()>30){
                               outtakeHammer.setPosition(.1);
                               scoreDelay.reset();
                               scoringState++;
                           }
                           break;
                       case 4:
                           if(scoreDelay.milliseconds()>30){
                               transfer.setPosition(outtake2);
                               scoreDelay.reset();
                               scoringState++;
                           }
                           break;
                       case 7:
                           if( scoreDelay.milliseconds()>30){
                               transfer.setPosition(outtake3);
                               scoreDelay.reset();
                               scoringState++;
                           }
                           break;
                       case 10:
                           scoring=false;
                           transfer.setPosition(intake1);
                           outtake.setPower(0);
                           scoringState = 1;
                           break;
                   }




               }
               else if(Objects.equals(artifactOrder, "PGP")){
                   switch (scoringState){
                       case 1:
                           outtake.setPower(1);
                           transfer.setPosition(outtake1);
                           scoreDelay.reset();
                           scoringState++;
                           break;
                       case 2:
                       case 5:
                       case 8:
                           if(scoreDelay.milliseconds()>50){
                               outtakeHammer.setPosition((double) 110 /300);
                               scoreDelay.reset();
                               scoringState++;
                           }
                           break;
                       case 3:
                       case 6:
                       case 9:
                           if(scoreDelay.milliseconds()>30){
                               outtakeHammer.setPosition(.1);
                               scoreDelay.reset();
                               scoringState++;
                           }
                           break;
                       case 4:
                           if(scoreDelay.milliseconds()>30){
                               transfer.setPosition(outtake3);
                               scoreDelay.reset();
                               scoringState++;
                           }
                           break;
                       case 7:
                           if( scoreDelay.milliseconds()>30){
                               transfer.setPosition(outtake2);
                               scoreDelay.reset();
                               scoringState++;
                           }
                           break;
                       case 10:
                           scoring=false;
                           transfer.setPosition(intake1);
                           outtake.setPower(0);
                           scoringState = 1;
                           break;
                   }
               }
               else if(Objects.equals(artifactOrder, "GPP")){
                   switch (scoringState){
                       case 1:
                           outtake.setPower(1);
                           transfer.setPosition(outtake3);
                           scoreDelay.reset();
                           scoringState++;
                           break;
                       case 2:
                       case 5:
                       case 8:
                           if(scoreDelay.milliseconds()>50){
                               outtakeHammer.setPosition((double) 110 /300);
                               scoreDelay.reset();
                               scoringState++;
                           }
                           break;
                       case 3:
                       case 6:
                       case 9:
                           if(scoreDelay.milliseconds()>30){
                               outtakeHammer.setPosition(.1);
                               scoreDelay.reset();
                               scoringState++;
                           }
                           break;
                       case 4:
                           if(scoreDelay.milliseconds()>30){
                               transfer.setPosition(outtake2);
                               scoreDelay.reset();
                               scoringState++;
                           }
                           break;
                       case 7:
                           if( scoreDelay.milliseconds()>30){
                               transfer.setPosition(outtake1);
                               scoreDelay.reset();
                               scoringState++;
                           }
                           break;
                       case 10:
                           scoring=false;
                           transfer.setPosition(intake1);
                           outtake.setPower(0);
                           scoringState = 1;
                           break;
                   }
               }
           }
           else if(Objects.equals(MOTIFPATTERN, "PGP")){
                if(Objects.equals(artifactOrder, "PPG")){
                    switch (scoringState){
                        case 1:
                            outtake.setPower(1);
                            transfer.setPosition(outtake1);
                            scoreDelay.reset();
                            scoringState++;
                            break;
                        case 2:
                        case 5:
                        case 8:
                            if(scoreDelay.milliseconds()>50){
                                outtakeHammer.setPosition((double) 110 /300);
                                scoreDelay.reset();
                                scoringState++;
                            }
                            break;
                        case 3:
                        case 6:
                        case 9:
                            if(scoreDelay.milliseconds()>30){
                                outtakeHammer.setPosition(.1);
                                scoreDelay.reset();
                                scoringState++;
                            }
                            break;
                        case 4:
                            if(scoreDelay.milliseconds()>30){
                                transfer.setPosition(outtake3);
                                scoreDelay.reset();
                                scoringState++;
                            }
                            break;
                        case 7:
                            if( scoreDelay.milliseconds()>30){
                                transfer.setPosition(outtake2);
                                scoreDelay.reset();
                                scoringState++;
                            }
                            break;
                        case 10:
                            scoring=false;
                            transfer.setPosition(intake1);
                            outtake.setPower(0);
                            scoringState = 1;
                            break;
                    }
               }
                else if(Objects.equals(artifactOrder, "PGP")){
                    switch (scoringState){
                        case 1:
                            outtake.setPower(1);
                            transfer.setPosition(outtake1);
                            scoreDelay.reset();
                            scoringState++;
                            break;
                        case 2:
                        case 5:
                        case 8:
                            if(scoreDelay.milliseconds()>50){
                                outtakeHammer.setPosition((double) 110 /300);
                                scoreDelay.reset();
                                scoringState++;
                            }
                            break;
                        case 3:
                        case 6:
                        case 9:
                            if(scoreDelay.milliseconds()>30){
                                outtakeHammer.setPosition(.1);
                                scoreDelay.reset();
                                scoringState++;
                            }
                            break;
                        case 4:
                            if(scoreDelay.milliseconds()>30){
                                transfer.setPosition(outtake2);
                                scoreDelay.reset();
                                scoringState++;
                            }
                            break;
                        case 7:
                            if( scoreDelay.milliseconds()>30){
                                transfer.setPosition(outtake3);
                                scoreDelay.reset();
                                scoringState++;
                            }
                            break;
                        case 10:
                            scoring=false;
                            transfer.setPosition(intake1);
                            outtake.setPower(0);
                            scoringState = 1;
                            break;
                    }

               }
                else if(Objects.equals(artifactOrder, "GPP")){
                    switch (scoringState){
                        case 1:
                            outtake.setPower(1);
                            transfer.setPosition(outtake3);
                            scoreDelay.reset();
                            scoringState++;
                            break;
                        case 2:
                        case 5:
                        case 8:
                            if(scoreDelay.milliseconds()>50){
                                outtakeHammer.setPosition((double) 110 /300);
                                scoreDelay.reset();
                                scoringState++;
                            }
                            break;
                        case 3:
                        case 6:
                        case 9:
                            if(scoreDelay.milliseconds()>30){
                                outtakeHammer.setPosition(.1);
                                scoreDelay.reset();
                                scoringState++;
                            }
                            break;
                        case 4:
                            if(scoreDelay.milliseconds()>30){
                                transfer.setPosition(outtake2);
                                scoreDelay.reset();
                                scoringState++;
                            }
                            break;
                        case 7:
                            if( scoreDelay.milliseconds()>30){
                                transfer.setPosition(outtake1);
                                scoreDelay.reset();
                                scoringState++;
                            }
                            break;
                        case 10:
                            scoring=false;
                            transfer.setPosition(intake1);
                            outtake.setPower(0);
                            scoringState = 1;
                            break;
                    }
               }

           }
           else if(Objects.equals(MOTIFPATTERN, "GPP")){
                if(Objects.equals(artifactOrder, "PPG")){
                    switch (scoringState){
                        case 1:
                            outtake.setPower(1);
                            transfer.setPosition(outtake3);
                            scoreDelay.reset();
                            scoringState++;
                            break;
                        case 2:
                        case 5:
                        case 8:
                            if(scoreDelay.milliseconds()>50){
                                outtakeHammer.setPosition((double) 110 /300);
                                scoreDelay.reset();
                                scoringState++;
                            }
                            break;
                        case 3:
                        case 6:
                        case 9:
                            if(scoreDelay.milliseconds()>30){
                                outtakeHammer.setPosition(.1);
                                scoreDelay.reset();
                                scoringState++;
                            }
                            break;
                        case 4:
                            if(scoreDelay.milliseconds()>30){
                                transfer.setPosition(outtake2);
                                scoreDelay.reset();
                                scoringState++;
                            }
                            break;
                        case 7:
                            if( scoreDelay.milliseconds()>30){
                                transfer.setPosition(outtake1);
                                scoreDelay.reset();
                                scoringState++;
                            }
                            break;
                        case 10:
                            scoring=false;
                            transfer.setPosition(intake1);
                            outtake.setPower(0);
                            scoringState = 1;
                            break;
                    }
               }
                else if(Objects.equals(artifactOrder, "PGP")){
                    switch (scoringState){
                        case 1:
                            outtake.setPower(1);
                            transfer.setPosition(outtake2);
                            scoreDelay.reset();
                            scoringState++;
                            break;
                        case 2:
                        case 5:
                        case 8:
                            if(scoreDelay.milliseconds()>50){
                                outtakeHammer.setPosition((double) 110 /300);
                                scoreDelay.reset();
                                scoringState++;
                            }
                            break;
                        case 3:
                        case 6:
                        case 9:
                            if(scoreDelay.milliseconds()>30){
                                outtakeHammer.setPosition(.1);
                                scoreDelay.reset();
                                scoringState++;
                            }
                            break;
                        case 4:
                            if(scoreDelay.milliseconds()>30){
                                transfer.setPosition(outtake1);
                                scoreDelay.reset();
                                scoringState++;
                            }
                            break;
                        case 7:
                            if( scoreDelay.milliseconds()>30){
                                transfer.setPosition(outtake3);
                                scoreDelay.reset();
                                scoringState++;
                            }
                            break;
                        case 10:
                            scoring=false;
                            transfer.setPosition(intake1);
                            outtake.setPower(0);
                            scoringState = 1;
                            break;
                    }
               }
                else if(Objects.equals(artifactOrder, "GPP")){
                    switch (scoringState){
                        case 1:
                            outtake.setPower(1);
                            transfer.setPosition(outtake1);
                            scoreDelay.reset();
                            scoringState++;
                            break;
                        case 2:
                        case 5:
                        case 8:
                            if(scoreDelay.milliseconds()>50){
                                outtakeHammer.setPosition((double) 110 /300);
                                scoreDelay.reset();
                                scoringState++;
                            }
                            break;
                        case 3:
                        case 6:
                        case 9:
                            if(scoreDelay.milliseconds()>30){
                                outtakeHammer.setPosition(.1);
                                scoreDelay.reset();
                                scoringState++;
                            }
                            break;
                        case 4:
                            if(scoreDelay.milliseconds()>30){
                                transfer.setPosition(outtake2);
                                scoreDelay.reset();
                                scoringState++;
                            }
                            break;
                        case 7:
                            if( scoreDelay.milliseconds()>30){
                                transfer.setPosition(outtake3);
                                scoreDelay.reset();
                                scoringState++;
                            }
                            break;
                        case 10:
                            scoring=false;
                            transfer.setPosition(intake1);
                            outtake.setPower(0);
                            scoringState = 1;
                            break;
                    }
               }
           }

        }
    }
    private void updateIntake(){
        if (intaking){
            switch (intakeState){
                case 1:
                    transfer.setPosition(intake1);
                    rightIntake.setPower(1);
                    leftIntake.setPower(1);
                    intakeDelay.reset();
                    intakeState++;
                    break;
                case 2:
                    if (intakeDelay.milliseconds()>333){
                        transfer.setPosition(intake2);
                        intakeDelay.reset();
                        intakeState++;
                    }
                    break;
                case 3:
                    if (intakeDelay.milliseconds()>333){
                        transfer.setPosition(intake3);
                        intakeDelay.reset();
                        intakeState++;
                    }
                    break;
                case 4:
                    if (intakeDelay.milliseconds()>333){
                        scoring = false;
                        scoringState = 1;
                        leftIntake.setPower(0);
                        rightIntake.setPower(0);
                    }
                    break;
            }
        }
    }

    private void buildPaths() {
        grabPickup1 =  ()-> follower.pathBuilder()
                .addPath( new BezierLine(follower::getPose, new Pose(40.5, 84)))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .addPoseCallback(new Pose(40.5,84), this::runIntake, .9)
                .addPath( new BezierLine(follower::getPose, pickup1Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine(follower::getPose, scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        grabPickup2 =()-> follower.pathBuilder()
                .addPath(new BezierLine(follower::getPose, new Pose(40.5,60)))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .addPoseCallback(new Pose(40.5,60), this::runIntake, .9)
                .addPath(new BezierLine(follower::getPose, pickup2Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine(follower::getPose, scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();
    }

    private void runIntake() {
        intaking=true;
    }

    private void initHardware() {
        transfer = hardwareMap.get(Servo.class, "transfer");
        outtakeHammer = hardwareMap.get(Servo.class, "outtakeHammer");

        outtake = hardwareMap.get(DcMotor.class, "outtake");
        outtake.setMode(RUN_WITHOUT_ENCODER);
        outtake.setZeroPowerBehavior(BRAKE);

        rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");
        rightIntake.setMode(RUN_WITHOUT_ENCODER);

        leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        leftIntake.setMode(RUN_WITHOUT_ENCODER);
        leftIntake.setDirection(REVERSE);



        driveFrontLeft = hardwareMap.get(DcMotor.class, "driveFrontLeft");
        driveFrontLeft.setMode(STOP_AND_RESET_ENCODER);
        driveFrontLeft.setMode(RUN_WITHOUT_ENCODER);
        driveFrontLeft.setZeroPowerBehavior(BRAKE);
        driveFrontLeft.setDirection(REVERSE);

        driveFrontRight = hardwareMap.get(DcMotor.class, "driveFrontRight");
        driveFrontRight.setMode(STOP_AND_RESET_ENCODER);
        driveFrontRight.setMode(RUN_WITHOUT_ENCODER);
        driveFrontRight.setZeroPowerBehavior(BRAKE);

        driveBackLeft = hardwareMap.get(DcMotor.class, "driveBackLeft");
        driveBackLeft.setMode(STOP_AND_RESET_ENCODER);
        driveBackLeft.setMode(RUN_WITHOUT_ENCODER);
        driveBackLeft.setZeroPowerBehavior(BRAKE);
        driveBackLeft.setDirection(REVERSE);

        driveBackRight = hardwareMap.get(DcMotor.class, "driveBackRight");
        driveBackRight.setMode(STOP_AND_RESET_ENCODER);
        driveBackRight.setMode(RUN_WITHOUT_ENCODER);
        driveBackRight.setZeroPowerBehavior(BRAKE);





    }

    private void setApriltag(){
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                .setLensIntrinsics(237.835, 237.835, 328.272, 237.727)
                // ... these parameters are fx, fy, cx, cy.
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag);
        visionportal = builder.build();
    }

    private List AprilTagDetection(){
        String motif = "unfound";
        Pose position = null;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection: currentDetections) {
            if (detection.metadata != null) {
                // Only use tags that don't have Obelisk in them
                if (!detection.metadata.name.contains("Obelisk")) {
                    position = new Pose(detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES),
                            FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
                }else{
                    motif = detection.metadata.name.replace("Obelisk-","");
                }

        }
        

    }
        return Arrays.asList(motif, position);
    }
}
