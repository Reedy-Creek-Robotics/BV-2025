package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "servo tune")
public class servoTune extends LinearOpMode {
    Servo transfer;
    ElapsedTime buttonDebounce = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    DcMotor motor;
    int i;
    @Override
    public void runOpMode() throws InterruptedException {
        transfer = hardwareMap.get(Servo.class, "transfer");
        motor =   hardwareMap.get(DcMotor.class, "outtake");
        motor.setMode(RUN_WITHOUT_ENCODER);
        waitForStart();
        i=100;
        transfer.setPosition((double) 0 /1800);
        while(opModeIsActive()){
            if(gamepad1.dpad_up && buttonDebounce.milliseconds()>100){
                buttonDebounce.reset();
                i++;
            } else if (gamepad1.dpad_down && buttonDebounce.milliseconds()>100) {
                buttonDebounce.reset();
                i--;
            }
            telemetry.addData("transfer pos", i);
            updateTelemetry(telemetry);
            transfer.setPosition((double) i /1800);
            if(gamepad1.a){
                motor.setPower(1);
            }else{
                motor.setPower(0);
            }
        }
    }
}
