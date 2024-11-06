package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commandgroups.ArmPositionCommandGroup;
import org.firstinspires.ftc.teamcode.commandgroups.AutonBasketReadyCommandGroup;
import org.firstinspires.ftc.teamcode.commandgroups.AutonParkTouchCommandGroup;
import org.firstinspires.ftc.teamcode.commandgroups.AutonSampleReadyCommandGroup;
import org.firstinspires.ftc.teamcode.commandgroups.AutonSpecimenFloorGrabCommandGroup;
import org.firstinspires.ftc.teamcode.commandgroups.AutonSpecimenHangCommandGroup;
import org.firstinspires.ftc.teamcode.commands.ClawCommand;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utility.PoseStorage;

@Autonomous(group = "Autonomous",name="Specimens", preselectTeleOp="Manual Drive")
public class AutonSpecimens extends LinearOpMode {

    enum State {
        TRAJ_1,   // Move to submersible with pre-loaded specimen, raising arm to high chamber ready level
        TRAJ_2,   // Attempt specimen hang
        TRAJ_3,   // Move to floor specimen, lowering arm to ready position
        TRAJ_4,   // Attempt specimen grab
        TRAJ_5,   // Move to submersible, raising arm to high chamber ready level
        TRAJ_6,   // Attempt specimen hang
        TRAJ_7,   // Move to floor specimen location, lowering arm to ready position
        TRAJ_8,   // Attempt specimen grab
        TRAJ_9,   // Move to submersible, raising arm to high chamber ready level
        TRAJ_10,  // Attempt specimen hang
        TRAJ_11,  // Park, lowering arm to travel position
        IDLE      // Prior to start
    }

    State currentState = State.IDLE;

    private final RobotHardware robot = RobotHardware.getInstance();

    //Define our start position (against wall, facing right, back of robot lined up with tile teeth near net zone tape)
    Pose2d startPose = new Pose2d(8.5, -63, Math.toRadians(90));

    //Define all trajectory start/end poses
    Pose2d floorGrabPose=new Pose2d(30.5, -59.5, Math.toRadians(0));
    Pose2d preParkPose=new Pose2d(34, -58, Math.toRadians(90));
    Pose2d parkPose=new Pose2d(57, -60, Math.toRadians(90));
    Pose2d specimenScorePose1=new Pose2d(-5.5, -31.5, Math.toRadians(90));
    Pose2d specimenScorePose2=new Pose2d(-3.5, -31.5, Math.toRadians(90));
    Pose2d specimenScorePose3=new Pose2d(-1.5, -31.5, Math.toRadians(90));

    //Define delays for arm actions
    double delayScoreReady=0.2;
    double delayScoreAttempt=2.5;
    double delayGrabReady=0.8;
    double delayGrabAttempt=1.6;

    private double loopTime = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();

        robot.init(hardwareMap,telemetry);

        //Set starting pose
        robot.drive.setPoseEstimate(startPose);

        //Define all trajectories

        //(1) Move to submersible with pre-loaded specimen, raising arm to high chamber ready level
        TrajectorySequence trajectory1=robot.drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new ArmPositionCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw,4));
                })
                .lineToLinearHeading(specimenScorePose1)
                .waitSeconds(delayScoreReady)
                .build();

       //(2) Attempt specimen hang
       TrajectorySequence trajectory2=robot.drive.trajectorySequenceBuilder(specimenScorePose1)
                .waitSeconds(delayScoreAttempt)
               .build();

        //(3) Move to floor specimen, lowering arm to ready position
        TrajectorySequence trajectory3=robot.drive.trajectorySequenceBuilder(specimenScorePose1)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new ArmPositionCommandGroup(robot.armAngle,1.0,robot.armWinch,1.0,robot.wrist,1.0,robot.claw,1.0,6));
                })
                .lineToLinearHeading(floorGrabPose)
                .waitSeconds(delayGrabReady)
                .build();

        //(4) Attempt specimen grab
        TrajectorySequence trajectory4=robot.drive.trajectorySequenceBuilder(floorGrabPose)
                .waitSeconds(delayGrabAttempt)
                .build();

        //(5) Move to submersible, raising arm to high chamber ready level
        TrajectorySequence trajectory5=robot.drive.trajectorySequenceBuilder(floorGrabPose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new ArmPositionCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw,4));
                })
                .lineToLinearHeading(specimenScorePose2)
                .waitSeconds(delayScoreReady)
                .build();

        //(6) Attempt specimen hang
        TrajectorySequence trajectory6=robot.drive.trajectorySequenceBuilder(specimenScorePose2)
                .waitSeconds(delayScoreAttempt)
                .build();

        //(7) Move to floor specimen location, lowering arm to ready position
        TrajectorySequence trajectory7=robot.drive.trajectorySequenceBuilder(specimenScorePose2)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new ArmPositionCommandGroup(robot.armAngle,1.0,robot.armWinch,1.0,robot.wrist,1.0,robot.claw,1.0,6));
                })
                .lineToLinearHeading(floorGrabPose)
                .build();

        //(8) Attempt specimen grab
        TrajectorySequence trajectory8=robot.drive.trajectorySequenceBuilder(floorGrabPose)
                .waitSeconds(delayGrabAttempt)
                .build();

        //(9) Move to submersible, raising arm to high chamber ready level
        TrajectorySequence trajectory9=robot.drive.trajectorySequenceBuilder(floorGrabPose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new ArmPositionCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw,4));
                })
                .lineToLinearHeading(specimenScorePose3)
                .waitSeconds(delayScoreReady)
                .build();

        //(10) Attempt specimen hang
        TrajectorySequence trajectory10=robot.drive.trajectorySequenceBuilder(specimenScorePose3)
                .waitSeconds(delayScoreAttempt)
                .build();

        //(11) Park
        TrajectorySequence trajectory11=robot.drive.trajectorySequenceBuilder(specimenScorePose3)
                .lineToLinearHeading(preParkPose)
                .lineToLinearHeading(parkPose)
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

        boolean actionFlag=false;

        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();

            switch (currentState) {
                case TRAJ_1:
                    if(!robot.drive.isBusy()) {
                        currentState= State.TRAJ_2;
                        robot.drive.followTrajectorySequenceAsync(trajectory2);
                    }
                    break;
                case TRAJ_2:
                    if(!actionFlag) {
                        CommandScheduler.getInstance().schedule(new AutonSpecimenHangCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                        actionFlag=true;
                    }

                    if(!robot.drive.isBusy()) {
                        actionFlag=false;
                        currentState= State.TRAJ_3;
                        robot.drive.followTrajectorySequenceAsync(trajectory3);
                    }
                    break;
                case TRAJ_3:
                    if(!robot.drive.isBusy()) {
                        currentState= State.TRAJ_4;
                        robot.drive.followTrajectorySequenceAsync(trajectory4);
                    }
                    break;
                case TRAJ_4:
                    if(!actionFlag) {
                        CommandScheduler.getInstance().schedule(new AutonSpecimenFloorGrabCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                        actionFlag=true;
                    }

                    if(!robot.drive.isBusy()) {
                        actionFlag=false;
                        currentState= State.TRAJ_5;
                        robot.drive.followTrajectorySequenceAsync(trajectory5);
                    }
                    break;
                case TRAJ_5:
                    if(!robot.drive.isBusy()) {
                        currentState= State.TRAJ_6;
                        robot.drive.followTrajectorySequenceAsync(trajectory6);
                    }
                    break;
                case TRAJ_6:
                    if(!actionFlag) {
                        CommandScheduler.getInstance().schedule(new AutonSpecimenHangCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                        actionFlag=true;
                    }

                    if(!robot.drive.isBusy()) {
                        actionFlag=false;
                        currentState= State.TRAJ_7;
                        robot.drive.followTrajectorySequenceAsync(trajectory7);
                    }
                    break;
                case TRAJ_7:
                    if(!robot.drive.isBusy()) {
                        currentState= State.TRAJ_8;
                        robot.drive.followTrajectorySequenceAsync(trajectory8);
                    }
                    break;
                case TRAJ_8:
                    if(!actionFlag) {
                        CommandScheduler.getInstance().schedule(new AutonSpecimenFloorGrabCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                        actionFlag=true;
                    }

                    if(!robot.drive.isBusy()) {
                        actionFlag=false;
                        currentState= State.TRAJ_9;
                        robot.drive.followTrajectorySequenceAsync(trajectory9);
                    }
                    break;
                case TRAJ_9:
                    if(!robot.drive.isBusy()) {
                        currentState= State.TRAJ_10;
                        robot.drive.followTrajectorySequenceAsync(trajectory10);
                    }
                    break;
                case TRAJ_10:
                    if(!actionFlag) {
                        CommandScheduler.getInstance().schedule(new AutonSpecimenHangCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                        actionFlag=true;
                    }

                    if(!robot.drive.isBusy()) {
                        actionFlag=false;
                        currentState= State.TRAJ_11;
                        robot.drive.followTrajectorySequenceAsync(trajectory11);
                    }
                    break;
                case TRAJ_11:
                    if(!robot.drive.isBusy()) {
                        currentState= State.IDLE;
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
