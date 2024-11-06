package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commandgroups.ArmPositionCommandGroup;
import org.firstinspires.ftc.teamcode.commandgroups.AutonBasketDropCommandGroup;
import org.firstinspires.ftc.teamcode.commandgroups.AutonBasketReadyCommandGroup;
import org.firstinspires.ftc.teamcode.commandgroups.AutonParkTouchCommandGroup;
import org.firstinspires.ftc.teamcode.commandgroups.AutonSampleDropCommandGroup;
import org.firstinspires.ftc.teamcode.commandgroups.AutonSampleGrabCommandGroup;
import org.firstinspires.ftc.teamcode.commandgroups.AutonSampleReadyCommandGroup;
import org.firstinspires.ftc.teamcode.commandgroups.AutonSpecimenHangCommandGroup;
import org.firstinspires.ftc.teamcode.commands.ClawCommand;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utility.PoseStorage;

@Autonomous(group = "Autonomous",name="Samples", preselectTeleOp="Manual Drive")
public class AutonSamples extends LinearOpMode {

    enum State {
        TRAJ_1,   // Drive to basket and raise arm
        TRAJ_1a,   // Drop sample and return arm to travel position
        TRAJ_2,   // Move forward to right-most floor sample
        TRAJ_2a,   // Attempt grab and return arm to travel position
        TRAJ_3,   // Drive to basket and raise arm
        TRAJ_3a,   // Drop sample and return arm to travel position
        TRAJ_4,   // Move forward to center floor sample
        TRAJ_4a,   // Attempt grab and return arm to travel position
        TRAJ_5,   // Drive to basket and raise arm
        TRAJ_5a,   // Drop sample and return arm to travel position
        TRAJ_6,   // Move forward to left-side floor sample
        TRAJ_6a,   // Attempt grab and return arm to travel position
        TRAJ_7,   // Drive to basket and raise arm
        TRAJ_7a,   // Drop sample and return arm to travel position
        TRAJ_8,   //Park in observation zone
        IDLE      //Prior to start
    }

    State currentState = State.IDLE;

    private final RobotHardware robot = RobotHardware.getInstance();

    //Define our start position (against wall, facing right, back of robot lined up with tile teeth near net zone tape)
    Pose2d startPose = new Pose2d(-39, -63, Math.toRadians(0));

    //Define all trajectory start/end poses
    Pose2d firstBasketPose=new Pose2d(-57.5, -54.5, Math.toRadians(45));
    Pose2d basketPose=new Pose2d(-57.5, -54.5, Math.toRadians(45));
    Pose2d rightSamplePose=new Pose2d(-49.5, -45, Math.toRadians(90));
    Pose2d centerSamplePose=new Pose2d(-59.5, -45, Math.toRadians(90));
    Pose2d leftSamplePose=new Pose2d(-59, -42.5, Math.toRadians(118));
    Pose2d parkPose=new Pose2d(-28, 0, Math.toRadians(0));

    //Define delays for arm actions
    double delayBasketDrop=1.4;
    double delaySampleGrab=1.4;
    double delayDriveBasket=0.6;

    private double loopTime = 0.0;
    private double actionTime=0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();

        robot.init(hardwareMap,telemetry);

        //Set starting pose
        robot.drive.setPoseEstimate(startPose);

        //===== DEFINE TRAJECTORIES =====

        //(1) Drive to basket and raise arm
        TrajectorySequence trajectory1=robot.drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new AutonBasketReadyCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                })
                .setReversed(true) //Sets backwards movement
                .lineToLinearHeading(firstBasketPose)
                .build();

        //(2) Drive to right sample and lower arm to ready position
        TrajectorySequence trajectory2=robot.drive.trajectorySequenceBuilder(firstBasketPose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new AutonSampleReadyCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                })
                .lineToLinearHeading(rightSamplePose)
                .build();

        //(3) Raise arm and drive to basket
        TrajectorySequence trajectory3=robot.drive.trajectorySequenceBuilder(rightSamplePose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new AutonBasketReadyCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                })
                .setReversed(true) //Sets backwards movement
                .lineToLinearHeading(basketPose)
                .waitSeconds(delayDriveBasket)
                .build();

        //(4) Drive to middle sample and lower arm to ready position
        TrajectorySequence trajectory4=robot.drive.trajectorySequenceBuilder(basketPose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new AutonSampleReadyCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                })
                .lineToLinearHeading(centerSamplePose)
                .build();

        //(5) Raise arm and drive to basket
        TrajectorySequence trajectory5=robot.drive.trajectorySequenceBuilder(centerSamplePose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new AutonBasketReadyCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                })
                .setReversed(true) //Sets backwards movement
                .lineToLinearHeading(basketPose)
                .waitSeconds(delayDriveBasket)
                .build();

        //(6) Drive to left sample and lower arm to ready position
        TrajectorySequence trajectory6=robot.drive.trajectorySequenceBuilder(basketPose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new AutonSampleReadyCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                })
                .lineToLinearHeading(leftSamplePose)
                .build();

        //(7) Raise arm and drive to basket
        TrajectorySequence trajectory7=robot.drive.trajectorySequenceBuilder(leftSamplePose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new AutonBasketReadyCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                })
                .setReversed(true) //Sets backwards movement
                .lineToLinearHeading(basketPose)
                .waitSeconds(delayDriveBasket)
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

        boolean actionFlag=false;

        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();

            switch (currentState) {
                //(1) Drive to basket and raise arm
                case TRAJ_1:
                    if(!robot.drive.isBusy()) {
                        currentState=State.TRAJ_1a;
                        //robot.drive.followTrajectorySequenceAsync(trajectory1a);
                    }
                    break;

                //(1a) Drop sample and return arm to travel position
                case TRAJ_1a:
                    if(!actionFlag) {
                        CommandScheduler.getInstance().schedule(new AutonSampleDropCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                        actionFlag=true;
                        actionTime = System.nanoTime();
                    }

                    if((System.nanoTime()-actionTime)/1000000000>delayBasketDrop) {
                    //if(!robot.drive.isBusy()) {
                        actionFlag=false;
                        currentState=State.TRAJ_2;
                        robot.drive.followTrajectorySequenceAsync(trajectory2);
                    }
                    break;

                //(2) Drive to right sample and lower arm to ready position
                case TRAJ_2:
                    if(!robot.drive.isBusy()) {
                        currentState=State.TRAJ_2a;
                        //robot.drive.followTrajectorySequenceAsync(trajectory2a);
                    }
                    break;

                //(2a) Attempt grab and return to travel position
                case TRAJ_2a:
                    if(!actionFlag) {
                        CommandScheduler.getInstance().schedule(new AutonSampleGrabCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                        actionFlag=true;
                        actionTime = System.nanoTime();
                    }

                    if((System.nanoTime()-actionTime)/1000000000>delaySampleGrab) {
                        //if(!robot.drive.isBusy()) {
                        actionFlag=false;
                        currentState=State.TRAJ_3;
                        robot.drive.followTrajectorySequenceAsync(trajectory3);
                    }
                    break;

                //(3) Raise arm and drive to basket
                case TRAJ_3:
                    if(!robot.drive.isBusy()) {
                        currentState=State.TRAJ_3a;
                        //robot.drive.followTrajectorySequenceAsync(trajectory3a);
                    }
                    break;

                //(3a) Drop sample and return to travel position
                case TRAJ_3a:
                    if(!actionFlag) {
                        CommandScheduler.getInstance().schedule(new AutonSampleDropCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                        actionFlag=true;
                        actionTime = System.nanoTime();
                    }

                    if((System.nanoTime()-actionTime)/1000000000>delayBasketDrop) {
                        //if(!robot.drive.isBusy()) {
                        actionFlag=false;
                        currentState=State.TRAJ_4;
                        robot.drive.followTrajectorySequenceAsync(trajectory4);
                    }
                    break;

                //(4) Drive to middle sample and lower arm to ready position
                case TRAJ_4:
                    if(!robot.drive.isBusy()) {
                        currentState=State.TRAJ_4a;
                        //robot.drive.followTrajectorySequenceAsync(trajectory4a);
                    }
                    break;

                //(4a) Attempt grab and return to travel position
                case TRAJ_4a:
                    if(!actionFlag) {
                        CommandScheduler.getInstance().schedule(new AutonSampleGrabCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                        actionFlag=true;
                        actionTime = System.nanoTime();
                    }

                    if((System.nanoTime()-actionTime)/1000000000>delaySampleGrab) {
                        //if(!robot.drive.isBusy()) {
                        actionFlag=false;
                        currentState=State.TRAJ_5;
                        robot.drive.followTrajectorySequenceAsync(trajectory5);
                    }
                    break;

                //(5) Raise arm and drive to basket
                case TRAJ_5:
                    if(!robot.drive.isBusy()) {
                        currentState=State.TRAJ_5a;
                        //robot.drive.followTrajectorySequenceAsync(trajectory5a);
                    }
                    break;

                //(5a) Drop sample and return arm to travel position
                case TRAJ_5a:
                    if(!actionFlag) {
                        CommandScheduler.getInstance().schedule(new AutonSampleDropCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                        actionFlag=true;
                        actionTime = System.nanoTime();
                    }

                    if((System.nanoTime()-actionTime)/1000000000>delayBasketDrop) {
                        //if(!robot.drive.isBusy()) {
                        actionFlag=false;
                        currentState=State.TRAJ_6;
                        robot.drive.followTrajectorySequenceAsync(trajectory6);
                    }
                    break;

                case TRAJ_6:
                    if(!robot.drive.isBusy()) {
                        currentState=State.TRAJ_6a;
                        //robot.drive.followTrajectorySequenceAsync(trajectory6a);
                    }
                    break;

                case TRAJ_6a:
                    if(!actionFlag) {
                        CommandScheduler.getInstance().schedule(new AutonSampleGrabCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                        actionFlag=true;
                        actionTime = System.nanoTime();
                    }

                    if((System.nanoTime()-actionTime)/1000000000>delaySampleGrab) {
                        //if(!robot.drive.isBusy()) {
                        actionFlag=false;
                        currentState=State.TRAJ_7;
                        robot.drive.followTrajectorySequenceAsync(trajectory7);
                    }
                    break;

                case TRAJ_7:
                    if(!robot.drive.isBusy()) {
                        currentState=State.TRAJ_7a;
                        //robot.drive.followTrajectorySequenceAsync(trajectory7a);
                    }
                    break;

                case TRAJ_7a:
                    if(!actionFlag) {
                        CommandScheduler.getInstance().schedule(new AutonSampleDropCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                        actionFlag=true;
                        actionTime = System.nanoTime();
                    }

                    if((System.nanoTime()-actionTime)/1000000000>delayBasketDrop) {
                        //if(!robot.drive.isBusy()) {
                        actionFlag=false;
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
            telemetry.addData("Trajectory: ", currentState);
            telemetry.addData("x: ", poseEstimate.getX());
            telemetry.addData("y: ", poseEstimate.getY());
            telemetry.addData("heading: ", poseEstimate.getHeading());
            loopTime = loop;
            telemetry.update();
        }
    }
}
