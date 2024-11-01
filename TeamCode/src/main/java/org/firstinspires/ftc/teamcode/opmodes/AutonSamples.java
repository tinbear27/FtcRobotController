package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commandgroups.ArmPositionCommandGroup;
import org.firstinspires.ftc.teamcode.commandgroups.AutonBasketDropCommandGroup;
import org.firstinspires.ftc.teamcode.commandgroups.AutonBasketReadyCommandGroup;
import org.firstinspires.ftc.teamcode.commandgroups.AutonParkTouchCommandGroup;
import org.firstinspires.ftc.teamcode.commandgroups.AutonSampleGrabCommandGroup;
import org.firstinspires.ftc.teamcode.commandgroups.AutonSampleReadyCommandGroup;
import org.firstinspires.ftc.teamcode.commands.ClawCommand;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utility.PoseStorage;

@Autonomous(group = "Autonomous",name="Samples")
public class AutonSamples extends LinearOpMode {

    enum State {
        TRAJ_1,   // Move to basket with pre-loaded sample
        TRAJ_1a,   // Move to basket with pre-loaded sample
        TRAJ_1b,   // Move to basket with pre-loaded sample
        TRAJ_2,   // Move forward to right-most floor sample
        TRAJ_2a,   // Move forward to right-most floor sample
        TRAJ_2b,   // Move forward to right-most floor sample
        TRAJ_3,   // Move to basket
        TRAJ_3a,   // Move to basket
        TRAJ_3b,   // Move to basket
        TRAJ_4,   // Move forward to center floor sample
        TRAJ_4a,   // Move forward to center floor sample
        TRAJ_4b,   // Move forward to center floor sample
        TRAJ_5,   // Move to basket
        TRAJ_5a,   // Move to basket
        TRAJ_5b,   // Move to basket
        TRAJ_6,   // Move forward to left-side floor sample
        TRAJ_6a,   // Move forward to left-side floor sample
        TRAJ_6b,   // Move forward to left-side floor sample
        TRAJ_7,   // Move to basket
        TRAJ_7a,   // Move to basket
        TRAJ_7b,   // Move to basket
        TRAJ_8,   //Park in observation zone
        IDLE      //Prior to start
    }

    State currentState = State.IDLE;

    private final RobotHardware robot = RobotHardware.getInstance();
    //private Drivetrain drive;

    //Define our start position (against wall, facing right, back of robot lined up with tile teeth near net zone tape)
    Pose2d startPose = new Pose2d(-39, -63, Math.toRadians(0));

    //Define all trajectory start/end poses
    Pose2d basketPose=new Pose2d(-56.5, -52.5, Math.toRadians(45));
    Pose2d rightSamplePose=new Pose2d(-49.5, -46, Math.toRadians(90));
    Pose2d centerSamplePose=new Pose2d(-59.5, -46, Math.toRadians(90));
    Pose2d leftSamplePose=new Pose2d(-59, -43, Math.toRadians(118));
    Pose2d parkPose=new Pose2d(-28, 0, Math.toRadians(0));

    //Define delays for arm actions
    double delayBasketDrop=0.8;
    double delaySampleReady=0.3;
    double delaySampleGrab=0.6;

    private double loopTime = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();

        robot.init(hardwareMap,telemetry);

        //Set starting pose
        robot.drive.setPoseEstimate(startPose);

        //Define all trajectories

        //(1) Drive to basket
        TrajectorySequence trajectory1=robot.drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new AutonBasketReadyCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                })
                .setReversed(true) //Sets backwards movement
                .lineToLinearHeading(basketPose)
                .build();

       //(1a) Drop sample
       TrajectorySequence trajectory1a=robot.drive.trajectorySequenceBuilder(basketPose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new ClawCommand(robot.claw, Constants.Arm.Claw.CLAW_OPEN));
                })
                .waitSeconds(0.5)
               .build();

        //(1b) Arm to travel position
        TrajectorySequence trajectory1b=robot.drive.trajectorySequenceBuilder(basketPose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new ArmPositionCommandGroup(robot.armAngle,0.0,robot.armWinch,0.5,robot.wrist,0.3,null,0.0,1));
                })
                .waitSeconds(delayBasketDrop)
                .build();

        //(2) Drive to right sample and lower arm to ready position
        TrajectorySequence trajectory2=robot.drive.trajectorySequenceBuilder(basketPose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new AutonSampleReadyCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                })
                .lineToLinearHeading(rightSamplePose)
                .waitSeconds(delaySampleReady)
                .build();

        //(2a) Attempt grab
        TrajectorySequence trajectory2a=robot.drive.trajectorySequenceBuilder(rightSamplePose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new ArmPositionCommandGroup(robot.armAngle,0.0,robot.armWinch,0.0,robot.wrist,0.0,robot.claw,0.2,23));
                })
                .waitSeconds(0.8)
                .build();

        //(2b) Return arm to travel position
        TrajectorySequence trajectory2b=robot.drive.trajectorySequenceBuilder(rightSamplePose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new ArmPositionCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,null,1));
                })
                .waitSeconds(delaySampleGrab)
                .build();

        //(3) Raise arm and drive to basket
        TrajectorySequence trajectory3=robot.drive.trajectorySequenceBuilder(rightSamplePose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new AutonBasketReadyCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                })
                .setReversed(true) //Sets backwards movement
                .lineToLinearHeading(basketPose)
                .build();

        //(3a) Drop sample
        TrajectorySequence trajectory3a=robot.drive.trajectorySequenceBuilder(basketPose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new ClawCommand(robot.claw, Constants.Arm.Claw.CLAW_OPEN));
                })
                .waitSeconds(0.5)
                .build();

        //(3b) Return arm to travel position
        TrajectorySequence trajectory3b=robot.drive.trajectorySequenceBuilder(basketPose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new ArmPositionCommandGroup(robot.armAngle,0.0,robot.armWinch,0.5,robot.wrist,0.3,null,0.0,1));
                })
                .waitSeconds(delayBasketDrop)
                .build();

        //(4) Drive to middle sample and lower arm to ready position
        TrajectorySequence trajectory4=robot.drive.trajectorySequenceBuilder(basketPose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new AutonSampleReadyCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                })
                .lineToLinearHeading(centerSamplePose)
                .waitSeconds(delaySampleReady)
                .build();

        //(4a) Attempt grab
        TrajectorySequence trajectory4a=robot.drive.trajectorySequenceBuilder(centerSamplePose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new ArmPositionCommandGroup(robot.armAngle,0.0,robot.armWinch,0.0,robot.wrist,0.0,robot.claw,0.2,23));
                })
                .waitSeconds(0.8)
                .build();

        //(4b) Return arm to travel position
        TrajectorySequence trajectory4b=robot.drive.trajectorySequenceBuilder(centerSamplePose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new ArmPositionCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,null,1));
                })
                .waitSeconds(delaySampleGrab)
                .build();

        //(5) Raise arm and drive to basket
        TrajectorySequence trajectory5=robot.drive.trajectorySequenceBuilder(centerSamplePose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new AutonBasketReadyCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                })
                .setReversed(true) //Sets backwards movement
                .lineToLinearHeading(basketPose)
                .build();

        //(5a) Drop sample
        TrajectorySequence trajectory5a=robot.drive.trajectorySequenceBuilder(basketPose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new ClawCommand(robot.claw, Constants.Arm.Claw.CLAW_OPEN));
                })
                .waitSeconds(0.5)
                .build();

        //(5b) Return arm to travel position
        TrajectorySequence trajectory5b=robot.drive.trajectorySequenceBuilder(basketPose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new ArmPositionCommandGroup(robot.armAngle,0.0,robot.armWinch,0.5,robot.wrist,0.3,null,0.0,1));
                })
                .waitSeconds(delayBasketDrop)
                .build();

        //(6) Drive to left sample and lower arm to ready position
        TrajectorySequence trajectory6=robot.drive.trajectorySequenceBuilder(basketPose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new AutonSampleReadyCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                })
                .lineToLinearHeading(leftSamplePose)
                .waitSeconds(delaySampleReady)
                .build();

        //(6a) Attempt grab
        TrajectorySequence trajectory6a=robot.drive.trajectorySequenceBuilder(leftSamplePose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new ArmPositionCommandGroup(robot.armAngle,0.0,robot.armWinch,0.0,robot.wrist,0.0,robot.claw,0.2,23));
                })
                .waitSeconds(0.8)
                .build();

        //(6b) Return arm to travel position
        TrajectorySequence trajectory6b=robot.drive.trajectorySequenceBuilder(centerSamplePose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new ArmPositionCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,null,1));
                })
                .waitSeconds(delaySampleGrab)
                .build();


        //(7) Raise arm and drive to basket
        TrajectorySequence trajectory7=robot.drive.trajectorySequenceBuilder(leftSamplePose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new AutonBasketReadyCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                })
                .setReversed(true) //Sets backwards movement
                .lineToLinearHeading(basketPose)
                .build();

        //(7a) Drop sample
        TrajectorySequence trajectory7a=robot.drive.trajectorySequenceBuilder(basketPose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new ClawCommand(robot.claw, Constants.Arm.Claw.CLAW_OPEN));
                })
                .waitSeconds(0.5)
                .build();

        //(7b) Return arm to travel position
        TrajectorySequence trajectory7b=robot.drive.trajectorySequenceBuilder(basketPose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new ArmPositionCommandGroup(robot.armAngle,0.0,robot.armWinch,0.5,robot.wrist,0.3,null,0.0,1));
                })
                .waitSeconds(delayBasketDrop)
                .build();

        //(8) Park
        TrajectorySequence trajectory8=robot.drive.trajectorySequenceBuilder(basketPose)
                .lineToLinearHeading(parkPose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new AutonParkTouchCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                })
                .build();

        //Zero arm angle and winch
        robot.armZero();
        robot.holdStartPosition();

        while(opModeInInit()) {
            robot.armAngle.periodic();
            robot.armWinch.periodic();

            telemetry.addData("Global Arm Position: ",robot.ARM_POSITION);
            telemetry.addData("Arm Angle (target): ",robot.armAngle.getTarget());
            telemetry.addData("Arm Angle (position): ",robot.armAngle.getPosition());
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        //robot.armAngle.setActiveRunMode(); //Change arm angle motor back to correct runmode
        currentState = State.TRAJ_1;
        robot.drive.followTrajectorySequenceAsync(trajectory1);

        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();

            switch (currentState) {
                case TRAJ_1:
                    if(!robot.drive.isBusy()) {
                        currentState=State.TRAJ_1a;
                        robot.drive.followTrajectorySequenceAsync(trajectory1a);
                    }
                    break;
                case TRAJ_1a:
                    if(!robot.drive.isBusy()) {
                        currentState=State.TRAJ_1b;
                        robot.drive.followTrajectorySequenceAsync(trajectory1b);
                    }
                    break;
                case TRAJ_1b:
                    if(!robot.drive.isBusy()) {
                        currentState=State.TRAJ_2;
                        robot.drive.followTrajectorySequenceAsync(trajectory2);
                    }
                    break;
                case TRAJ_2:
                    if(!robot.drive.isBusy()) {
                        currentState=State.TRAJ_2a;
                        robot.drive.followTrajectorySequenceAsync(trajectory2a);
                    }
                    break;
                case TRAJ_2a:
                    if(!robot.drive.isBusy()) {
                        currentState=State.TRAJ_2b;
                        robot.drive.followTrajectorySequenceAsync(trajectory2b);
                    }
                    break;
                case TRAJ_2b:
                    if(!robot.drive.isBusy()) {
                        currentState=State.TRAJ_3;
                        robot.drive.followTrajectorySequenceAsync(trajectory3);
                    }
                    break;
                case TRAJ_3:
                    if(!robot.drive.isBusy()) {
                        currentState=State.TRAJ_3a;
                        robot.drive.followTrajectorySequenceAsync(trajectory3a);
                    }
                    break;
                case TRAJ_3a:
                    if(!robot.drive.isBusy()) {
                        currentState=State.TRAJ_3b;
                        robot.drive.followTrajectorySequenceAsync(trajectory3b);
                    }
                    break;
                case TRAJ_3b:
                    if(!robot.drive.isBusy()) {
                        currentState=State.TRAJ_4;
                        robot.drive.followTrajectorySequenceAsync(trajectory4);
                    }
                    break;
                case TRAJ_4:
                    if(!robot.drive.isBusy()) {
                        currentState=State.TRAJ_4a;
                        robot.drive.followTrajectorySequenceAsync(trajectory4a);
                    }
                    break;
                case TRAJ_4a:
                    if(!robot.drive.isBusy()) {
                        currentState=State.TRAJ_4b;
                        robot.drive.followTrajectorySequenceAsync(trajectory4b);
                    }
                    break;
                case TRAJ_4b:
                    if(!robot.drive.isBusy()) {
                        currentState=State.TRAJ_5;
                        robot.drive.followTrajectorySequenceAsync(trajectory5);
                    }
                    break;

                case TRAJ_5:
                    if(!robot.drive.isBusy()) {
                        currentState=State.TRAJ_5a;
                        robot.drive.followTrajectorySequenceAsync(trajectory5a);
                    }
                    break;
                case TRAJ_5a:
                    if(!robot.drive.isBusy()) {
                        currentState=State.TRAJ_5b;
                        robot.drive.followTrajectorySequenceAsync(trajectory5b);
                    }
                    break;
                case TRAJ_5b:
                    if(!robot.drive.isBusy()) {
                        currentState=State.TRAJ_6;
                        robot.drive.followTrajectorySequenceAsync(trajectory6);
                    }
                    break;
                case TRAJ_6:
                    if(!robot.drive.isBusy()) {
                        currentState=State.TRAJ_6a;
                        robot.drive.followTrajectorySequenceAsync(trajectory6a);
                    }
                    break;
                case TRAJ_6a:
                    if(!robot.drive.isBusy()) {
                        currentState=State.TRAJ_6b;
                        robot.drive.followTrajectorySequenceAsync(trajectory6b);
                    }
                    break;
                case TRAJ_6b:
                    if(!robot.drive.isBusy()) {
                        currentState=State.TRAJ_7;
                        robot.drive.followTrajectorySequenceAsync(trajectory7);
                    }
                    break;
                case TRAJ_7:
                    if(!robot.drive.isBusy()) {
                        currentState=State.TRAJ_7a;
                        robot.drive.followTrajectorySequenceAsync(trajectory7a);
                    }
                    break;
                case TRAJ_7a:
                    if(!robot.drive.isBusy()) {
                        currentState=State.TRAJ_7b;
                        robot.drive.followTrajectorySequenceAsync(trajectory7b);
                    }
                    break;
                case TRAJ_7b:
                    if(!robot.drive.isBusy()) {
                        currentState=State.TRAJ_8;
                        robot.drive.followTrajectorySequenceAsync(trajectory8);
                    }
                    break;
                case TRAJ_8:
                    if(!robot.drive.isBusy()) {
                        currentState=State.IDLE;
                    }
                    break;
                case IDLE:
                    break;
            }

            robot.drive.update();

            // Read current pose and save to storage
            Pose2d poseEstimate = robot.drive.getPoseEstimate();
            PoseStorage.currentPose = poseEstimate;

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
