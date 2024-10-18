package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commandgroups.AutonBasketDropCommandGroup;
import org.firstinspires.ftc.teamcode.commandgroups.AutonBasketReadyCommandGroup;
import org.firstinspires.ftc.teamcode.commandgroups.AutonParkTouchCommandGroup;
import org.firstinspires.ftc.teamcode.commandgroups.AutonSampleGrabCommandGroup;
import org.firstinspires.ftc.teamcode.commandgroups.AutonSampleReadyCommandGroup;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "Autonomous",name="Samples")
public class AutonSamples extends LinearOpMode {

    enum State {
        TRAJ_1,   // Move to basket with pre-loaded sample
        TRAJ_2,   // Move forward to right-most floor sample
        TRAJ_3,   // Move to basket
        TRAJ_4,   // Move forward to center floor sample
        TRAJ_5,   // Move to basket
        TRAJ_6,   // Move forward to left-side floor sample
        TRAJ_7,   // Move to basket
        TRAJ_8,   //Park in observation zone
        IDLE      //Prior to start
    }

    State currentState = State.IDLE;

    private final RobotHardware robot = RobotHardware.getInstance();
    private Drivetrain drive;

    //Define our start position (against wall, facing right, back of robot lined up with tile teeth near net zone tape)
    Pose2d startPose = new Pose2d(-39, -63, Math.toRadians(0));

    //Define all trajectory start/end poses
    Pose2d basketPose=new Pose2d(-56.5, -56.5, Math.toRadians(45));
    Pose2d rightSamplePose=new Pose2d(-49.5, -44, Math.toRadians(90));
    Pose2d centerSamplePose=new Pose2d(-59.5, -44, Math.toRadians(90));
    Pose2d leftSamplePose=new Pose2d(-59, -43, Math.toRadians(120));
    Pose2d parkPose=new Pose2d(-28, 0, Math.toRadians(0));

    //Define delays for arm actions
    double delayBasketDrop=0.3;
    double delaySampleReady=0.0;
    double delaySampleGrab=0.7;

    private double loopTime = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();

        robot.init(hardwareMap);
        this.drive = new Drivetrain(hardwareMap);

        //Set starting pose
        drive.setPoseEstimate(startPose);

        //Define all trajectories
        TrajectorySequence trajectory1=drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new AutonBasketReadyCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                })
                .setReversed(true) //Sets backwards movement
                .lineToLinearHeading(basketPose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new AutonBasketDropCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                })
                .waitSeconds(delayBasketDrop)
                .build();

        TrajectorySequence trajectory2=drive.trajectorySequenceBuilder(basketPose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new AutonSampleReadyCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                })
                .lineToLinearHeading(rightSamplePose)
                .waitSeconds(delaySampleReady)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new AutonSampleGrabCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                })
                .waitSeconds(delaySampleGrab)
                .build();

        TrajectorySequence trajectory3=drive.trajectorySequenceBuilder(rightSamplePose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new AutonBasketReadyCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                })
                .setReversed(true) //Sets backwards movement
                .lineToLinearHeading(basketPose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new AutonBasketDropCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                })
                .waitSeconds(delayBasketDrop)
                .build();

        TrajectorySequence trajectory4=drive.trajectorySequenceBuilder(basketPose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new AutonSampleReadyCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                })
                .lineToLinearHeading(centerSamplePose)
                .waitSeconds(delaySampleReady)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new AutonSampleGrabCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                })
                .waitSeconds(delaySampleGrab)
                .build();

        TrajectorySequence trajectory5=drive.trajectorySequenceBuilder(centerSamplePose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new AutonBasketReadyCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                })
                .setReversed(true) //Sets backwards movement
                .lineToLinearHeading(basketPose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new AutonBasketDropCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                })
                .waitSeconds(delayBasketDrop)
                .build();

        TrajectorySequence trajectory6=drive.trajectorySequenceBuilder(basketPose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new AutonSampleReadyCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                })
                .lineToLinearHeading(leftSamplePose)
                .waitSeconds(delaySampleReady)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new AutonSampleGrabCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                })
                .waitSeconds(delaySampleGrab)
                .build();

        TrajectorySequence trajectory7=drive.trajectorySequenceBuilder(leftSamplePose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new AutonBasketReadyCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                })
                .setReversed(true) //Sets backwards movement
                .lineToLinearHeading(basketPose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new AutonBasketDropCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                })
                .waitSeconds(delayBasketDrop)
                .build();

        TrajectorySequence trajectory8=drive.trajectorySequenceBuilder(basketPose)
                .lineToLinearHeading(parkPose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new AutonParkTouchCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                })
                .build();

        while(opModeInInit()) {
            telemetry.addData("Global Arm Position: ",robot.ARM_POSITION);
            telemetry.addData("Arm Angle (position): ",robot.armAngle.getPosition());
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        robot.armAngle.setActiveRunMode(); //Change arm angle motor back to correct runmode
        currentState = State.TRAJ_1;
        drive.followTrajectorySequenceAsync(trajectory1);

        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
            robot.periodic();

            switch (currentState) {
                case TRAJ_1:
                    if(!drive.isBusy()) {
                        currentState=State.TRAJ_2;
                        drive.followTrajectorySequenceAsync(trajectory2);
                    }
                    break;
                case TRAJ_2:
                    if(!drive.isBusy()) {
                        currentState=State.TRAJ_3;
                        drive.followTrajectorySequenceAsync(trajectory3);
                    }
                    break;
                case TRAJ_3:
                    if(!drive.isBusy()) {
                        currentState=State.TRAJ_4;
                        drive.followTrajectorySequenceAsync(trajectory4);
                    }
                    break;
                case TRAJ_4:
                    if(!drive.isBusy()) {
                        currentState=State.TRAJ_5;
                        drive.followTrajectorySequenceAsync(trajectory5);
                    }
                    break;
                case TRAJ_5:
                    if(!drive.isBusy()) {
                        currentState=State.TRAJ_6;
                        drive.followTrajectorySequenceAsync(trajectory6);
                    }
                    break;
                case TRAJ_6:
                    if(!drive.isBusy()) {
                        currentState=State.TRAJ_7;
                        drive.followTrajectorySequenceAsync(trajectory7);
                    }
                    break;
                case TRAJ_7:
                    if(!drive.isBusy()) {
                        currentState=State.TRAJ_8;
                        drive.followTrajectorySequenceAsync(trajectory8);
                    }
                    break;
                case TRAJ_8:
                    if(!drive.isBusy()) {
                        currentState=State.IDLE;
                    }
                    break;
                case IDLE:
                    break;
            }

            drive.update();

            // Read current pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Print pose to telemetry
            double loop = System.nanoTime();
            telemetry.addData("Loop Speed (hz): ", 1000000000 / (loop - loopTime));
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            loopTime = loop;
            telemetry.update();
        }
    }
}
