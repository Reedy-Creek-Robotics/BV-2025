package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

import android.util.Log;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Auto Near Goal")
public  class RedAutoNearGoal extends OpMode {
    DcMotorEx outtake;
    DcMotor transfer;
    Follower follower;

    IMU imu;



    int autoState=0;
    ElapsedTime autoTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    int close=0, open=180/300;

    AutoPaths.RedNearGoal path;

    @Override
    public void init() {
        initHardware();
        follower=Constants.createFollower(hardwareMap);
       follower.setStartingPose( new Pose(117, 136, Math.toRadians(180)));
        path = new AutoPaths.RedNearGoal(follower);

    }

    @Override
    public void loop() {
        follower.update();
        autoStateHandlerNear();
        manageTelemetry();
        updateTelemetry(telemetry);
    }

    private void initHardware(){
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        outtake.setMode(RUN_USING_ENCODER);

        transfer = hardwareMap.get(DcMotor.class, "transfer");
        transfer.setDirection(FORWARD);
        transfer.setMode(RUN_WITHOUT_ENCODER);
        transfer.setZeroPowerBehavior(BRAKE);
    }

    private void autoStateHandlerNear(){
        switch (autoState){
            case 0:
                outtake.setVelocity(1525);
                follower.followPath(path.Score1.get());
                autoState++;
                autoTimer.reset();
                break;
            case 1:
            case 5:
            case 9:
                if(!follower.isBusy()&&autoTimer.milliseconds()>3000){
                    transfer.setPower(1);
                    autoTimer.reset();
                    autoState++;
                }
                break;
            case 2:
                if(autoTimer.milliseconds()>2500){
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
        telemetry.addData("Pose", follower.getPose());

    }
}
