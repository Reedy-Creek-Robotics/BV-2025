package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static com.sun.tools.doclint.Entity.not;

import android.annotation.SuppressLint;
import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
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

@Autonomous(name = "Blue Auto Start near goal")
public  class Auto extends OpMode {
    DcMotor outtake, transfer;
    Servo trigger;

    Follower follower;

    private Position cameraPosition = new Position(DistanceUnit.INCH,
            9, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
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

    final double open = 0,
            close =  1;

    Paths path;

    ElapsedTime autoTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    int autoState = 0;
    @Override
    public void init() {
        initHardware();
        initPosition();
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(new Pose(0,0,0));

    }

    @Override
    public void init_loop() {
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
                        initPose=new Pose(detection.robotPose.getPosition().x,
                                detection.robotPose.getPosition().y,
                                detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS),
                                FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
                        follower.setPose(initPose);
                        telemetry.addData("pedropathing place", initPose);
                        telemetry.addLine(String.valueOf(follower.poseTracker.getPose()));
                    }
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));


                }

        }
        path = new Paths(follower, initPose);
            updateTelemetry(telemetry);
            telemetry.addLine(path.pickup1.toString());

    }

    @Override
    public void start() {
        follower.setStartingPose(initPose);

    }

    @Override
    public void loop() {
        follower.update();
        autoStateHandler();
        telemetry.addData("Path State", autoState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        updateTelemetry(telemetry);
        endPose = follower.getPose();
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




        initPose = new Pose(0,0,0);
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
        switch (autoState){
            case 0:
                follower.followPath(path.Score1);
                autoTimer.reset();
                autoState++;
                break;
            case 1:
            case 4:
            case 7:
                if(!follower.isBusy()){
                    transfer.setPower(0);
                    trigger.setPosition(open);
                    outtake.setPower(.75  );
                    autoTimer.reset();
                    autoState++;
                }
                break;
            case 2:
            case 5:
            case 8:
                if(autoTimer.milliseconds()>3000){
                    transfer.setPower(.7);
                    autoState++;
                    autoTimer.reset();
                }
                break;
            case 3:
                if(autoTimer.milliseconds()>2000){
                    outtake.setPower(0);
                    trigger.setPosition(close);
                    follower.followPath(path.pickup1);
                    autoState++;
                    autoTimer.reset();
                }
                break;
            case 6:
                if(autoTimer.milliseconds()>2000){
                    outtake.setPower(0);
                    trigger.setPosition(close);
                    follower.followPath(path.pickup2);
                    autoState++;
                    autoTimer.reset();
                }



        }
    }

    public static class Paths {
        public PathChain Score1;

        public PathChain pickup1;
        public PathChain pickup2;

        public Paths(Follower follower, Pose initPose) {
            Score1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    initPose,

                                    new Pose(67.000, 81.000)
                            )
                    ).setLinearHeadingInterpolation(initPose.getHeading(), Math.toRadians(135))

                    .build();

             pickup1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(67.000, 81.000),

                                    new Pose(41.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .addPath(
                            new BezierLine(
                                    new Pose(41.000, 84.000),

                                    new Pose(19.000, 84.000)
                            )
                    ).setConstantHeadingInterpolation(180).addPath(
                             new BezierLine(
                                     new Pose(19.000, 84.000),

                                     new Pose(67.000, 81.000)
                             )
                     ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();



            pickup2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(67.000, 81.000),

                                    new Pose(42.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .addPath(
                            new BezierLine(
                                    new Pose(42.000, 60.000),

                                    new Pose(18.000, 60.000)
                            )
                    ).setTangentHeadingInterpolation().addPath(
                            new BezierLine(
                                    new Pose(18.000, 60.000),

                                    new Pose(67.000, 81.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();
        }
    }


}
