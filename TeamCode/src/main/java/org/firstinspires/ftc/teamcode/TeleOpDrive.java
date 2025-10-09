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
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@TeleOp(name = "TeleOP Drive")
public class TeleOpDrive extends LinearOpMode {
    private final Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);
    DcMotor driveFrontLeft, driveFrontRight, driveBackRight, driveBackLeft, outtake, leftIntake, rightIntake;

    Servo transfer, outtakeHammer;
    IMU imu;
    private Follower follower;
    PathChain score;
    ElapsedTime buttonDebounce;
    Gamepad gamepad1 = new Gamepad();

    AprilTagProcessor aprilTag;
    private VisionPortal visionportal;

    private String MOTIFPATTERN;
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
    private int scoringState=1;
    private boolean firstClick = false;


    @Override
    public void runOpMode() throws InterruptedException {
        MOTIFPATTERN = Auto.MOTIFPATTERN;
        Pose initPose;
        buttonDebounce = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        initHardware();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Auto.endPose);
        createPaths();
        waitForStart();
        while(opModeIsActive()){
            follower.update();
            updateScore();
            processControl();
        }

    }
    private void initHardware(){
        outtake = hardwareMap.get(DcMotor.class, "outtake");
        outtake.setMode(RUN_WITHOUT_ENCODER);
        outtake.setZeroPowerBehavior(BRAKE);

        rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");

        rightIntake.setMode(RUN_WITHOUT_ENCODER);

        leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        leftIntake.setMode(RUN_WITHOUT_ENCODER);
        leftIntake.setDirection(REVERSE);

        driveFrontLeft = hardwareMap.get(DcMotor.class,  "driveFrontLeft");
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

        outtakeHammer = hardwareMap.get(Servo.class, "outtakeHammer");

        // IMU TODO: replace this with the pinpoint sensor IMU later.
         imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

    }
    private void processControl() {
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (gamepad1.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        driveFrontLeft.setPower(frontLeftPower);
        driveBackLeft.setPower(backLeftPower);
        driveFrontRight.setPower(frontRightPower);
        driveBackRight.setPower(backRightPower);

        // now do buttons
        if(gamepad1.x && buttonDebounce.milliseconds()>250){
            buttonDebounce.reset();
            follower.followPath(score);
            scoring=true;
            firstClick = true;
        }
        if(gamepad1.x && buttonDebounce.milliseconds()>250&&firstClick){
            scoring = false;
            firstClick = false;
            follower.breakFollowing();
        }


    }

    private void updateScore() {
       if(scoring &&!follower.isBusy()){
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
/*
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

*/
    }

    private void createPaths(){
        score = follower.pathBuilder()
                .addPath(new BezierLine(follower::getPose, new Pose(60, 85)))
                .setLinearHeadingInterpolation(follower.getHeading(), Math.toRadians(135))
                .build();
    }
}
