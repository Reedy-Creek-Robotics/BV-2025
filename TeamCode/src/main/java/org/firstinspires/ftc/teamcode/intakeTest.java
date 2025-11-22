package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Intake Test", group = "tests")
public class intakeTest extends LinearOpMode {
    double
            intake1 = (double) 295 /1800,
            intake2 = (double) 424 /1800,
            intake3 = (double) 547 /1800,
            outtake2 = (double) 234/1800,
            outtake1 = (double) 100 /1800,
            outtake3 = (double) 362 /1800;
    private DcMotor rightIntake;
    private DcMotor leftIntake;
    private Servo transfer;
    private int state = 1;
    private ElapsedTime buttonDebounce = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        transfer.setPosition(intake1);
        rightIntake.setPower(1);
        leftIntake.setPower(1);
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.dpad_up&&buttonDebounce.milliseconds()>250){
                buttonDebounce.reset();
                state++;
            } else if(gamepad1.dpad_down&&buttonDebounce.milliseconds()>250){
                buttonDebounce.reset();
                state--;
            }
            if(state==1){
                transfer.setPosition(intake1);
            }else if(state ==2){
                transfer.setPosition(intake2);
            }else if(state==3){
                transfer.setPosition(intake3);
            }
            telemetry.addData("Intaking: ", state);
            telemetry.update();
        }
    }

    private void initHardware(){
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");
        rightIntake.setMode(RUN_WITHOUT_ENCODER);

        leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        leftIntake.setMode(RUN_WITHOUT_ENCODER);
        leftIntake.setDirection(REVERSE);

        transfer = hardwareMap.get(Servo.class, "transfer");

    }
}
