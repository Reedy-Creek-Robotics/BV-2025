package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

import android.util.Log;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue Far Triangle")
public  class BlueFarTriangle extends OpMode {
    DcMotorEx outtake;
    DcMotor transfer;
    Servo trigger;

    Follower follower;

    IMU imu;

    Limelight3A limelight;

    int autoState=0;
    ElapsedTime autoTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    int close=0, open=180/300;

    AutoPaths.BlueFarTriangle path;

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
        follower.setStartingPose( new Pose(117, 136, Math.toRadians(180)));
        limelight.start();
    }

    @Override
    public void init_loop() {
        Pose botPose = new Pose(144-27,144-8, Math.toRadians(180));
        if (botPose!=null) {

            if (botPose.getX() < 72){
                if(botPose.getY()>72){
                    PathFollowing=3;
                }else{
                    PathFollowing=4;
                }
            }else{
                if(botPose.getY()>72){
                    PathFollowing=1;
                }else{
                    PathFollowing=2;
                }
            }
        }
    }

    @Override
    public void start() {
//        limelight.pause();
////        if(PathFollowing==1){
//             path = new AutoPaths.BlueNearGoal(follower);
//        } else if (PathFollowing==2) {
//            path =new AutoPaths.BlueFarTriangle(follower);
//        } else if (PathFollowing==3) {
//            path  = new AutoPaths.RedNearGoal(follower);
//        }else{
//            path = new AutoPaths.RedFarTriangle(follower);
//        }

        follower.setStartingPose(new Pose(57,9, Math.toRadians(90)));
        path = new AutoPaths.BlueFarTriangle(follower);
    }

    @Override
    public void loop() {
        follower.update();
        autoStateHandlerNear();
        manageTelemetry();
        updateTelemetry(telemetry);
        Log.println(Log.DEBUG,"auto", path.getClass().toString() );


    }
    @Override
    public void stop() {
//        try (BufferedWriter writer = new BufferedWriter(new FileWriter("LastPose.csv"))) {
//            // Header (recommended)
//
//
//            // Data row
//            writer.write(
//                    follower.getPose().getX() + "," +
//                            follower.getPose().getY() + "," +
//                            follower.getPose().getHeading()
//            );
//        } catch (IOException e) {
//            throw new RuntimeException(e);
//        }
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




    }

    private  String escape(String value) {
        if (value == null) return "";
        if (value.contains(",") || value.contains("\"") || value.contains("\n")) {
            value = value.replace("\"", "\"\"");
            return "\"" + value + "\"";
        }
        return value;
    }




//    private void autoStateHandlerFar(){
//        switch (autoState){
//            case 0:
//                outtake.setVelocity(28*100);
//                autoTimer.reset();
//                autoState++;
//                break;
//            case 1:
//                if(autoTimer.milliseconds()>3000){
//                    trigger.setPosition(open);
//                    transfer.setPower(1);
//                    autoState++;
//                    autoTimer.reset();
//                }
//                break;
//            case 2:
//                if(autoTimer.milliseconds()>2500){
//                    trigger.setPosition(close);
//                    follower.followPath(path.pickup1);
//                    autoState++;
//                }
//                break;
//            case 3:
//            case 5:
//                if(!follower.isBusy()){
//                    trigger.setPosition(open);
//                    autoTimer.reset();
//                    autoState++;
//                }
//                break;
//            case 4:
//                if(autoTimer.milliseconds()>2500){
//                    trigger.setPosition(close);
//                    follower.followPath(path.pickup2);
//                    autoState++;
//                }
//                break;
//
//
//        }
//    }

    private void autoStateHandlerNear(){
        switch (autoState){
            case 0:
                outtake.setVelocity(2300);
                follower.followPath(path.Score1.get());
                autoState++;
                autoTimer.reset();
                break;
            case 1:
            case 5:
            case 9:
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
                if (!follower.isBusy()){
                    autoState++;
                    transfer.setPower(.46);
                    follower.followPath(path.pickup1_2, .5,true);
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    autoState++;
                    transfer.setPower(0);
                    follower.followPath(path.pickup1_3);
                    transfer.setPower(0);
                }break;

            case 6:
                if(autoTimer.milliseconds()>2500){
                    trigger.setPosition(close);
                    transfer.setPower(.0);
                    follower.followPath(path.pickup2);
                    autoState++;
                }
                break;
            case 7:
                if (!follower.isBusy()){
                    autoState++;
                    transfer.setPower(.46);
                    follower.followPath(path.pickup2_2,.5, true);
                }break;
            case 8:
                if(!follower.isBusy()){
                    trigger.setPosition(close);
                    transfer.setPower(0);
                    follower.followPath(path.pickup2_3);
                    autoState++;
                }break;

            case 10:
                if (autoTimer.milliseconds()>2500){
                    autoState++;
                    transfer.setPower(0);
                    follower.followPath(path.moveOffLine);
                    ;
                }break;
        }
    }

    private void manageTelemetry(){
        telemetry.addData("Path State", autoState);
        telemetry.addData("Path following ", PathFollowing);
        telemetry.addData("Pose", follower.getPose());

    }
}
