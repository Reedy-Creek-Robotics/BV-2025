package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import java.util.function.Supplier;
public class AutoPaths{
    PathChain pickup1;
    PathChain pickup2;
    PathChain Score1;
public static class BlueNearGoal  extends AutoPaths{
    public Supplier<PathChain> Score1;

    public PathChain pickup1;
    public PathChain pickup2;

    public Pose initPose;
    public BlueNearGoal(Follower follower) {
        Score1 = () ->follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(67, 81))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(135), 0.8))
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

public static class RedNearGoal extends AutoPaths{
        public PathChain Score1;

        public PathChain pickup1;
        public PathChain pickup2;

        public Pose initPose;
        public RedNearGoal(Follower follower) {
            Score1 = follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new Path(new BezierLine(new Pose(144-27,144-8, Math.toRadians(180)), new Pose(67, 81).mirror())))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(45), .8)
                    .build();

            pickup1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(67.000, 81.000).mirror(),

                                    new Pose(41.000, 84.000).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(41.000, 84.000).mirror(),

                                    new Pose(19.000, 84.000).mirror()
                            )
                    ).setConstantHeadingInterpolation(180).addPath(
                            new BezierLine(
                                    new Pose(19.000, 84.000).mirror(),

                                    new Pose(67.000, 81.000).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();



            pickup2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(67.000, 81.000).mirror(),

                                    new Pose(42.000, 60.000).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(42.000, 60.000).mirror(),

                                    new Pose(18.000, 60.000).mirror()
                            )
                    ).setTangentHeadingInterpolation().addPath(
                            new BezierLine(
                                    new Pose(18.000, 60.000).mirror(),

                                    new Pose(67.000, 81.000).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();
        }
    }

public static class BlueFarTriangle extends AutoPaths {
    public Supplier<PathChain> pickup1;

    public PathChain pickup2;


    public BlueFarTriangle(Follower follower) {
        pickup1 = ()->follower.pathBuilder().addPath(
                        new BezierLine(follower::getPose
                                ,

                                new Pose(42.000, 36.000)
                        )
                ).setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(180), .8)).addPath(
                        new BezierLine(
                                new Pose(42.000, 36.000),

                                new Pose(18.000, 36.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180)).addPath(
                        new BezierLine(
                                new Pose(18.000, 36.000),

                                new Pose(60.000, 12.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(111))

                .build();


        pickup2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(60.000, 12.000),

                                new Pose(42.000, 60.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(111), Math.toRadians(180)).addPath(
                        new BezierLine(
                                new Pose(42.000, 60.000),

                                new Pose(18.000, 60.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180)).addPath(
                        new BezierLine(
                                new Pose(18.000, 60.000),

                                new Pose(60.000, 12.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(111))
                .build();
    }


}

public static class RedFarTriangle extends AutoPaths{
    public Supplier<PathChain> pickup1;

    public PathChain pickup2;


    public RedFarTriangle(Follower follower) {
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