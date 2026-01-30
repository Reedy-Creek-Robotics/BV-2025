package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class outtakeTest extends OpMode {
    DcMotorEx outtake;
    int i=28;
    ElapsedTime buttonDebounce = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    @Override
    public void init() {
        outtake = hardwareMap.get(DcMotorEx.class,"outtake");
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        outtake.setVelocity(i);
        if(gamepad1.right_bumper&& buttonDebounce.milliseconds()>100){
            i+=50;
        } else if (gamepad1.left_bumper&& buttonDebounce.milliseconds()>100) {
            i-=50;
        }
        telemetry.addData("outtake velocity", outtake.getVelocity());
    }
}
