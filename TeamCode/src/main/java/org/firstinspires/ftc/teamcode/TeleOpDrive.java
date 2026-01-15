package org.firstinspires.ftc.teamcode;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

//import com.pedropathing.follower.Follower;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Objects;

@TeleOp(name = "TeleOP Drive with Pedropathing")
public class TeleOpDrive extends LinearOpMode {

    DcMotor outtake, transfer;

    Servo trigger;

    private Follower follower;
    private Supplier<PathChain> score;
    ElapsedTime buttonDebounce, scoreDebounce;

    final double  hi = 1,
                  lo = .75;

    final double open = 0,
                 close = 1;
    int scoreState = -1;
    private int transferState;
    private ElapsedTime transferDebounce = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void runOpMode() throws InterruptedException {
        buttonDebounce = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        scoreDebounce = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        initHardware();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(
                Auto.endPose != null ? Auto.endPose : new Pose(0, 0, 0)
        );

        createPaths();
        waitForStart();
        follower.startTeleopDrive(true);
        while(opModeIsActive()){
            follower.update();
            updateDriving();
            updateScoreState();
            manageTransferState();

        }

    }

    private void updateDriving() {
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,false);

//        if(gamepad1.xWasPressed()&& buttonDebounce.milliseconds()>250){
//            scoreState = 1;
//            buttonDebounce.reset();
//        }

        if(gamepad1.aWasPressed()){
            transfer.setPower(-1);
        }
        if(gamepad1.aWasReleased()){
            transfer.setPower(0);
        }
        if(gamepad1.yWasPressed()){
            transfer.setPower(1);
        } else if (gamepad1.yWasReleased()) {
            transferState = 0;


        }
        if(gamepad1.left_bumper){
            outtake.setPower(lo);
        } else if (gamepad1.right_bumper) {
            outtake.setPower(hi);
        }else{
            outtake.setPower(0);
        }

    }

    private void updateScoreState(){
        switch (scoreState){
            case 1:
                trigger.setPosition(open);
                transfer.setPower(0);
                outtake.setPower(1);
                scoreDebounce.reset();
                scoreState++;
                break;
            case 2:
                if(scoreDebounce.milliseconds()>1200){
                    transfer.setPower(1);
                    scoreDebounce.reset();
                    scoreState++;

                break;
                }
            case 3:
                if(scoreDebounce.milliseconds()>3000){
                    outtake.setPower(0);
                    trigger.setPosition(close);
                    scoreState=0;


                } break;

        }

    }

    private void initHardware(){
        outtake = hardwareMap.get(DcMotor.class, "outtake");
        outtake.setMode(RUN_WITHOUT_ENCODER);
        outtake.setZeroPowerBehavior(FLOAT);

        transfer = hardwareMap.get(DcMotor.class, "transfer");
        transfer.setMode(RUN_WITHOUT_ENCODER);
        transfer.setZeroPowerBehavior(BRAKE);
        transfer.setDirection(FORWARD);



        trigger = hardwareMap.get(Servo.class, "trigger");


    }


    private void createPaths(){
        score = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
     }

    private void manageTransferState(){
        switch (transferState){
            case 0:
                transfer.setPower(-1);
                transferState++;
                transferDebounce.reset();
                break;
            case 1:
                if(transferDebounce.milliseconds()>10){
                    transfer.setPower(0);
                    transferState = -1;
                }
                break;
        }
    }
}
