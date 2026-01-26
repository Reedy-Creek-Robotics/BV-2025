package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import android.annotation.SuppressLint;
import android.util.Log;
import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;
import java.util.function.Supplier;

@Autonomous(name = "all Autos")
public  class Auto extends OpMode {
    DcMotorEx outtake;
    DcMotor transfer;
    Servo trigger;

    Follower follower;

    IMU imu;

    Limelight3A limelight;

    int autoState=0;
    ElapsedTime autoTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    int close=0, open=180/300;

    AutoPaths path;

    int PathFollowing;
    /*
    * 1=blue near goal
    * 2=blue far triangle
    * 3=red near goal
    * 4=red far triangle
    */


    @Override
    public void init() {
        initHardware();
        follower=Constants.createFollower(hardwareMap);
        limelight.start();
    }

    @Override
    public void init_loop() {
        Pose botPose = robotPose();
        if (botPose!=null) {
            follower.setStartingPose(botPose);
            if (botPose.getX() < 72){
                if(botPose.getY()>72){
                    PathFollowing=1;
                }else{
                    PathFollowing=2;
                }
            }else{
                if(botPose.getY()>72){
                    PathFollowing=3;
                }else{
                    PathFollowing=4;
                }
            }
        }
    }

    @Override
    public void start() {
        limelight.pause();
        if(PathFollowing==1){
             path = new AutoPaths.BlueNearGoal(follower);
        } else if (PathFollowing==2) {
            path =new AutoPaths.BlueFarTriangle(follower);
        } else if (PathFollowing==3) {
            path  = new AutoPaths.RedNearGoal(follower);
        }else{
            path = new AutoPaths.RedFarTriangle(follower);
        }
    }

    @Override
    public void loop() {
        follower.update();
        if(PathFollowing==1||PathFollowing==3){
            autoStateHandlerNear();
        }else{
            autoStateHandlerFar();
        }
        manageTelemetry();
        updateTelemetry(telemetry);


    }
    @Override
    public void stop() {
        try (BufferedWriter writer = new BufferedWriter(new FileWriter("LastPose.csv"))) {
            // Header (recommended)


            // Data row
            writer.write(
                    follower.getPose().getX() + "," +
                            follower.getPose().getY() + "," +
                            follower.getPose().getHeading()
            );
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    private void initHardware(){
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        outtake.setMode(RUN_USING_ENCODER);

        transfer = hardwareMap.get(DcMotor.class, "transfer");
        transfer.setDirection(FORWARD);
        transfer.setMode(RUN_WITHOUT_ENCODER);
        transfer.setZeroPowerBehavior(BRAKE);

        trigger = hardwareMap.get(Servo.class,"trigger");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );


    }

    private  String escape(String value) {
        if (value == null) return "";
        if (value.contains(",") || value.contains("\"") || value.contains("\n")) {
            value = value.replace("\"", "\"\"");
            return "\"" + value + "\"";
        }
        return value;
    }

    private Pose robotPose(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
        LLResult result = limelight.getLatestResult();
        if(result!=null&& result.isValid()){
            Pose3D robotPose = result.getBotpose_MT2();
            return new Pose(robotPose.getPosition().x, robotPose.getPosition().y, robotPose.getOrientation().getYaw(AngleUnit.RADIANS), FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        }
        return null;
    }


    private void autoStateHandlerFar(){
        switch (autoState){
            case 0:
                outtake.setVelocity(28*100);
                autoTimer.reset();
                autoState++;
                break;
            case 1:
                if(autoTimer.milliseconds()>3000){
                    trigger.setPosition(open);
                    transfer.setPower(1);
                    autoState++;
                    autoTimer.reset();
                }
                break;
            case 2:
                if(autoTimer.milliseconds()>2500){
                    trigger.setPosition(close);
                    follower.followPath(path.pickup1);
                    autoState++;
                }
                break;
            case 3:
            case 5:
                if(!follower.isBusy()){
                    trigger.setPosition(open);
                    autoTimer.reset();
                    autoState++;
                }
                break;
            case 4:
                if(autoTimer.milliseconds()>2500){
                    trigger.setPosition(close);
                    follower.followPath(path.pickup2);
                    autoState++;
                }
                break;


        }
    }

    private void autoStateHandlerNear(){
        switch (autoState){
            case 0:
                outtake.setVelocity(28*95);
                follower.followPath(path.score1);
                autoState++;
                autoTimer.reset();
                break;
            case 1:
                if(!follower.isBusy()&&autoTimer.milliseconds()>3000){
                    transfer.setPower(1);
                    trigger.setPosition(open);
                    autoTimer.reset();
                    autoState++;
                }
                break;
            case 2:
                if(autoTimer.milliseconds()>2500){
                    trigger.setPosition(close);
                    follower.followPath(path.pickup1);
                    autoState++;
                }
                break;
            case 3:
            case 5:
                if(!follower.isBusy()){
                    transfer.setPower(1);
                    trigger.setPosition(open);
                    autoTimer.reset();
                    autoState++;
                }
                break;
            case 4:
                if(autoTimer.milliseconds()>2500){
                    trigger.setPosition(close);
                    follower.followPath(path.pickup2);
                    autoState++;
                }
                break;

        }
    }

    private void manageTelemetry(){
        telemetry.addData("Path State", autoState);
        telemetry.addData("Path following ", PathFollowing);
        telemetry.addData("Pose", follower.getPose());

    }
}
