package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp(name="outtake Test")
public class outtakeTest extends OpMode {
    DcMotorEx outtake;
    DcMotor transfer;
    int i=1550;
    boolean wasPressed = false;
    ElapsedTime buttonDebounce = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    @Override
    public void init() {
        outtake = hardwareMap.get(DcMotorEx.class,"outtake");
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        transfer.setDirection(FORWARD);
        transfer.setMode(RUN_WITHOUT_ENCODER);
        transfer.setZeroPowerBehavior(BRAKE);

    }

    @Override
    public void loop() {
        outtake.setVelocity(i);
        if(gamepad1.right_bumper&& buttonDebounce.milliseconds()>100){
            i+=50;
            buttonDebounce.reset();
        } else if (gamepad1.left_bumper&& buttonDebounce.milliseconds()>100) {
            i-=50;
            buttonDebounce.reset();
        }
        telemetry.addData("outtake velocity", outtake.getVelocity());
        telemetry.addData("wanted velocity", i);
        updateTelemetry(telemetry);
        if(gamepad1.a&&!wasPressed){
            transfer.setPower(1);

        }else{

            transfer.setPower(0);

        }
    }
}
