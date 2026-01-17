package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
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
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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

import java.util.List;
import java.util.function.Supplier;

@Autonomous(name = "Red Auto Start near goal")
public  class AutoRedFar extends OpMode {
    DcMotor outtake, transfer;
    Servo trigger;

    Follower follower;

    private final Position cameraPosition = new Position(DistanceUnit.INCH,
            9, 0, 0, 0);
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    Pose initPose;

    static Pose endPose;

    final double open = 0.85,
            close =  1;

    Paths path;

    ElapsedTime autoTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    int autoState = 0;
    @Override
    public void init() {
        initHardware();
        initPosition();
        follower = Constants.createFollower(hardwareMap);
        initPose  = new Pose(32,144-8.75,0);
        follower.setStartingPose(initPose);

        path = new Paths(follower);

    }

    @SuppressLint("DefaultLocale")
    public Pose robotPose() {
        Pose robotPose = null;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                // Only use tags that don't have Obelisk in them
                if (!detection.metadata.name.contains("Obelisk")) {
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getPosition().z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                    //Take that above and put it in the follower
                    robotPose=new Pose(detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS),
                            FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
                    follower.setPose(initPose);
                    telemetry.addData("pedropathing place", initPose);
                    telemetry.addLine(String.valueOf(follower.poseTracker.getPose()));
//
                }
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));


            }

        }

        updateTelemetry(telemetry);
        telemetry.addLine(path.pickup1.toString());
        return robotPose;

    }

    @Override
    public void start() {
        follower.setPose(robotPose());

        Log.println(Log.DEBUG, "automode", "start");
        Log.println(Log.DEBUG, "automode", String.valueOf(robotPose()));

    }

    @Override
    public void loop() {
        follower.update();
        autoStateHandler();
        follower.update();
        telemetry.addData("Path State", autoState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("isbusy", follower.isBusy());
        updateTelemetry(telemetry);
        endPose = follower.getPose();
        Pose pose  = robotPose();
        if(pose!= null){
            follower.setPose(pose);
        }
    }

    @SuppressLint("DefaultLocale")
    private void initPosition() {
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                .setLensIntrinsics(595.3753019, 597.10100376, 952.227276, 488.29700937)
                .build();
        aprilTag.setDecimation(3);

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);
        builder.setCameraResolution(new Size(1920, 1080));
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        visionPortal = builder.build();
        // Step through the list of detections and display info for each one.





    }

    private void initHardware(){
        outtake = hardwareMap.get(DcMotor.class, "outtake");
        outtake.setZeroPowerBehavior(BRAKE);
        outtake.setMode(RUN_WITHOUT_ENCODER);

        transfer = hardwareMap.get(DcMotor.class,"transfer");
        transfer.setDirection(REVERSE);
        transfer.setMode(RUN_WITHOUT_ENCODER);
        transfer.setZeroPowerBehavior(BRAKE);

        trigger = hardwareMap.get(Servo.class, "trigger");
    }

    private void autoStateHandler(){
        switch (autoState) {
            case 0:
            case 3:
            case 6:
                if(!follower.isBusy()) {
                    outtake.setPower(1);
                    transfer.setPower(0);
                    trigger.setPosition(close);
                    autoTimer.reset();
                    autoState++;
                }
                break;
            case 1:
            case 4:
            case 7:
                if(autoTimer.milliseconds()>3000){
                    transfer.setPower(.7);
                    trigger.setPosition(open);
                    autoTimer.reset();
                    autoState++;
                }
                break;
            case 2:
                if(autoTimer.milliseconds()>2000){
                    trigger.setPosition(close);
                    outtake.setPower(0);
                    follower.followPath(path.pickup1.get());
                    autoState++;
                }
                break;
            case 5:
                if(autoTimer.milliseconds()>2000){
                    trigger.setPosition(close);
                    outtake.setPower(0);
                    follower.followPath(path.pickup2);
                    autoState++;
                }
                break;

        }
    }

    public static class Paths {
        public Supplier<PathChain> pickup1;

        public PathChain pickup2;


        public Paths(Follower follower) {
            pickup1 = ()->follower.pathBuilder().addPath(
                            new BezierLine(follower::getPose
                                    ,

                                    new Pose(42.000, 36.000).mirror()
                            )
                    ).setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(0), .8)).addPath(
                            new BezierLine(
                                    new Pose(42.000, 36.000).mirror(),

                                    new Pose(18.000, 36.000).mirror()
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0)).addPath(
                            new BezierLine(
                                    new Pose(18.000, 36.000).mirror(),

                                    new Pose(60.000, 12.000).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(69))

                    .build();


            pickup2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(60.000, 12.000).mirror(),

                                    new Pose(42.000, 60.000).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(69), Math.toRadians(0)).addPath(
                            new BezierLine(
                                    new Pose(42.000, 60.000).mirror(),

                                    new Pose(18.000, 60.000).mirror()
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0)).addPath(
                            new BezierLine(
                                    new Pose(18.000, 60.000).mirror(),

                                    new Pose(60.000, 12.000).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(69))
                    .build();
        }
    }


}



