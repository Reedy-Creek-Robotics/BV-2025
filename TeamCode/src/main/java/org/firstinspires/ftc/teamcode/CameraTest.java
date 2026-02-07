package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@TeleOp(name = "cameraTest")
public class CameraTest extends OpMode {
    Limelight3A limelight;
    IMU imu;
    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
                )

                        )
                );
        limelight.start();


    }

    @Override
    public void loop() {
        limelight.updateRobotOrientation(imu.getRobotYawPitchRollAngles().getYaw());
        LLResult result = limelight.getLatestResult();
        Position botPose = result.getBotpose().getPosition().toUnit(DistanceUnit.INCH);
        Pose2D botPose2d = new Pose2D(DistanceUnit.INCH, botPose.x,
                botPose.toUnit(DistanceUnit.INCH).y, AngleUnit.RADIANS, result.getBotpose().getOrientation().getYaw(AngleUnit.RADIANS));
        Pose botPoseFTC = PoseConverter.pose2DToPose(botPose2d, FTCCoordinates.INSTANCE);
        Pose pedroPose = botPoseFTC.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        telemetry.addData("Pose3d", botPose);
        telemetry.addData("Pose2d", "x:"+botPose2d.getX(DistanceUnit.INCH)+", y:"+botPose2d.getY(DistanceUnit.INCH));
        telemetry.addData("Pedro Pose",pedroPose);
        telemetry.update();
    }

}
