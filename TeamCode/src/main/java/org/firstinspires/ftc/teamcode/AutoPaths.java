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
    public static class BlueNearGoal extends AutoPaths{
        public PathChain Score1;

        public PathChain pickup1, pickup1_2, pickup1_3;
        public PathChain pickup2, pickup2_2, pickup2_3;
        public Path moveOffLine;

        public Pose initPose;
        public BlueNearGoal(Follower follower) {
            Score1 =follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new BezierLine(new Pose(31,144-10), new Pose(44, 99)))
                    .setLinearHeadingInterpolation(0, Math.toRadians(135),.8)
                    .build();

            pickup1 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(49, 104),

                            new Pose(60, 84.000)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180)).build();

            pickup1_2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(60, 84.000),

                                    new Pose(17, 84.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180)).build();

            pickup1_3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(10, 84.000),

                                    new Pose(49, 104)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();



            pickup2 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(49, 104),

                            new Pose(50, 60)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180)).build();
            pickup2_2 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(50, 60),

                            new Pose(9, 60)
                    )
            ).setTangentHeadingInterpolation().build();

            pickup2_3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(7, 60),

                                    new Pose(46, 60)
                            )
                    ).setConstantHeadingInterpolation(180).addPath(
                            new BezierLine(
                                    new Pose(46,60),
                                    new Pose(49,104)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();

            moveOffLine = new Path(new BezierLine(new Pose(44, 99), new Pose(35,67)));

        }
    }

public static class RedNearGoal extends AutoPaths{
        public Supplier<PathChain> Score1;

        public PathChain pickup1, pickup1_2, pickup1_3;
        public PathChain pickup2, pickup2_2, pickup2_3;
        public Path moveOffLine;

        public Pose initPose;
        public RedNearGoal(Follower follower) {
            Score1 =()->follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new BezierLine(follower::getPose, new Pose(44, 99).mirror()))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45),.8))
                    .build();

            pickup1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(44, 99).mirror(),

                                    new Pose(43, 87.000).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0)).build();
            pickup1_2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(43, 87.000).mirror(),

                                    new Pose(10, 87.000).mirror()
                            )
                    )
                    .setConstantHeadingInterpolation(0).build();

            pickup1_3 = follower.pathBuilder()                .addPath(
                            new BezierLine(
                                    new Pose(10, 87.000).mirror(),

                                    new Pose(44, 99).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();



            pickup2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(44, 99).mirror(),

                                    new Pose(46, 63.000).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0)).build();
            pickup2_2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(46, 63.000).mirror(),

                                    new Pose(7, 63.000).mirror()
                            )
                    ).setTangentHeadingInterpolation().build();

            pickup2_3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(7, 63.000).mirror(),

                                    new Pose(46, 63).mirror()
                            )
                    ).setConstantHeadingInterpolation(0).addPath(
                            new BezierLine(
                                    new Pose(46,65).mirror(),
                                    new Pose(44,99).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            moveOffLine = new Path(new BezierLine(new Pose(44, 99).mirror(), new Pose(35,67).mirror()));

        }
    }

public static class BlueFarTriangle extends AutoPaths{
    public Supplier<PathChain> Score1;

    public PathChain pickup1, pickup1_2, pickup1_3;
    public PathChain pickup2, pickup2_2, pickup2_3;
    public PathChain moveOffLine;

    public Pose initPose;
    public BlueFarTriangle(Follower follower) {
        Score1 =()-> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new BezierLine(follower::getPose, new Pose(61.3, 15)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(135),.8))
                .build();

        pickup1 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(61.3, 15),

                        new Pose(43, 36)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(111), Math.toRadians(180)).build();
        pickup1_2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(43, 36),

                                new Pose(10, 36)
                        )
                )
                .setConstantHeadingInterpolation(180).build();

        pickup1_3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(10, 36),

                                new Pose(61.3, 15)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(111))
                .build();



        pickup2 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(61.3, 15),

                        new Pose(46, 63.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(111), Math.toRadians(180)).build();
        pickup2_2 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(46, 63.000),

                        new Pose(7, 63.000)
                )
        ).setTangentHeadingInterpolation().build();

        pickup2_3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(7, 63.000),

                                new Pose(46, 63)
                        )
                ).setConstantHeadingInterpolation(180).addPath(
                        new BezierLine(
                                new Pose(46,65),
                                new Pose(61.3,15)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(111))
                .build();

        moveOffLine =follower.pathBuilder().addPath( new Path(new BezierLine(new Pose(61.3, 15), new Pose(35,67)))).setLinearHeadingInterpolation(Math.toRadians(111),Math.toRadians(180)).build();

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