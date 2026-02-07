package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@TeleOp(group = "test")
public class hardwareTest extends LinearOpMode {

    DcMotor driveFrontLeft, driveFrontRight, driveBackRight, driveBackLeft, outtake, leftIntake, rightIntake;

//    Servo transfer;
//    IMU imu;
//    private Follower follower;
    PathChain score;
    ElapsedTime buttonDebounce = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    Gamepad gamepad1 = new Gamepad();
    int state = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();
        while(opModeIsActive()){
            switch(state){
                case 0:
                    driveBackLeft.setPower(-1);
                    driveBackRight.setPower(-1);
                    driveFrontLeft.setPower(-1);
                    driveFrontRight.setPower(-1);
                    //Derick, Do better.
                    buttonDebounce.reset();

                    state++;
                    break;
                case 1:
                    if(buttonDebounce.seconds()>5){

                        driveBackLeft.setPower(0);
                        driveBackRight.setPower(0);
                        driveFrontLeft.setPower(0);
                        driveFrontRight.setPower(0);
                    }
                    break;
            }
        }
    }


    private void initHardware() {


        driveFrontLeft = hardwareMap.get(DcMotor.class, "driveFrontLeft");
        driveFrontLeft.setMode(STOP_AND_RESET_ENCODER);
        driveFrontLeft.setMode(RUN_WITHOUT_ENCODER);
        driveFrontLeft.setZeroPowerBehavior(BRAKE);


        driveFrontRight = hardwareMap.get(DcMotor.class, "driveFrontRight");
        driveFrontRight.setMode(STOP_AND_RESET_ENCODER);
        driveFrontRight.setMode(RUN_WITHOUT_ENCODER);
        driveFrontRight.setZeroPowerBehavior(BRAKE);
        driveFrontRight.setDirection(REVERSE);
        //Derick You can do this.


        driveBackLeft = hardwareMap.get(DcMotor.class, "driveBackLeft");
        driveBackLeft.setMode(STOP_AND_RESET_ENCODER);
        driveBackLeft.setMode(RUN_WITHOUT_ENCODER);
        driveBackLeft.setZeroPowerBehavior(BRAKE);


        driveBackRight = hardwareMap.get(DcMotor.class, "driveBackRight");
        driveBackRight.setMode(STOP_AND_RESET_ENCODER);
        driveBackRight.setMode(RUN_WITHOUT_ENCODER);
        driveBackRight.setZeroPowerBehavior(BRAKE);
        driveBackRight.setDirection(REVERSE);
        //Do the gangam style dance 





    }



}
