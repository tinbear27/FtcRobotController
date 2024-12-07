package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commandgroups.ArmPositionCommandGroup;
import org.firstinspires.ftc.teamcode.commandgroups.AutonSpecimenFloorGrabCommandGroup;
import org.firstinspires.ftc.teamcode.commandgroups.AutonSpecimenHangCommandGroup;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utility.PoseStorage;

@Autonomous(group = "Autonomous",name="Specimens (5-cycle)", preselectTeleOp="Manual Drive")
public class AutonSpecimensAll extends LinearOpMode {

    enum State {
        TRAJ_1,   // Move to submersible with pre-loaded specimen, raising arm to high chamber ready level
        TRAJ_2,   // Attempt specimen hang

        TRAJ_3a,   // Move all samples to observation area
        TRAJ_3b,   // Move all samples to observation area
        TRAJ_3c,   // Move all samples to observation area
        TRAJ_3d,   // Move all samples to observation area
        TRAJ_3e,   // Move all samples to observation area
        TRAJ_3f,   // Move all samples to observation area

        TRAJ_4,   // Move to specimen location
        TRAJ_5,   // Attempt specimen grab
        TRAJ_6,   // Move to submersible, raising arm to high chamber ready level
        TRAJ_7,   // Attempt specimen hang

        TRAJ_8,   // Move to floor specimen location, lowering arm to ready position
        TRAJ_9,   // Attempt specimen grab
        TRAJ_10,   // Move to submersible, raising arm to high chamber ready level
        TRAJ_11,  // Attempt specimen hang

        TRAJ_12,   // Move to floor specimen location, lowering arm to ready position
        TRAJ_13,   // Attempt specimen grab
        TRAJ_14,   // Move to submersible, raising arm to high chamber ready level
        TRAJ_15,  // Attempt specimen hang

        TRAJ_16,   // Move to floor specimen location, lowering arm to ready position
        TRAJ_17,   // Attempt specimen grab
        TRAJ_18,   // Move to submersible, raising arm to high chamber ready level
        TRAJ_19,  // Attempt specimen hang

        TRAJ_20,  // Park, lowering arm to travel position
        IDLE      // Prior to start
    }

    State currentState = State.IDLE;

    private final RobotHardware robot = RobotHardware.getInstance();

    //Define our start position (against wall, facing right, back of robot lined up with tile teeth near net zone tape)
    Pose2d startPose = new Pose2d(8.5, -63, Math.toRadians(90));

    //Define all trajectory start/end poses
    Pose2d floorGrabPoseFirst=new Pose2d(53, -54.5, Math.toRadians(225));
    Pose2d floorGrabPose=new Pose2d(30.5, -59.5, Math.toRadians(0));
    Pose2d preParkPose=new Pose2d(34, -58, Math.toRadians(90));
    Pose2d parkPose=new Pose2d(57, -60, Math.toRadians(90));
    Pose2d specimenScorePose1=new Pose2d(-5, -35.5, Math.toRadians(90));
    Pose2d specimenScorePose2=new Pose2d(-1, -37, Math.toRadians(90));
    Pose2d specimenScorePose3=new Pose2d(3, -37, Math.toRadians(90));
    Pose2d specimenScorePose4=new Pose2d(7, -37, Math.toRadians(90));
    //Pose2d specimenScorePose5=new Pose2d(2.5, -37, Math.toRadians(90));

    //Define poses for samples to observation zone (splines)
    Pose2d samplePushPath1a=new Pose2d(36,-46,Math.toRadians(270)); //Move around submersible
    Pose2d samplePushPath1b=new Pose2d(46,-11,Math.toRadians(270)); //Position behind 1st sample (note heading value is not for robot heading, rather path)
    double samplePushPath1bPathAngle=Math.toRadians(32.0);

    Pose2d samplePushPath2=new Pose2d(46,-53,Math.toRadians(270)); //Push 1st sample to observation area

    Pose2d samplePushPath3=new Pose2d(55,-11,Math.toRadians(270)); //Position behind 2nd sample
    double samplePushPath3PathAngle=Math.toRadians(15);

    Pose2d samplePushPath4=new Pose2d(58,-60,Math.toRadians(270)); //Push 2nd sample to observation area
/*
    Pose2d samplePushPath5=new Pose2d(62,-8,Math.toRadians(270)); //Position behind 2nd sample
    double samplePushPath5PathAngle=Math.toRadians(15);

    Pose2d samplePushPath6=new Pose2d(62,-53,Math.toRadians(270)); //Push 2nd sample to observation area
*/

    //Define delays for arm actions
    double delayScoreReady=0.0;
    double delayScoreAttempt=1.0;
    double delayGrabAttempt=0.6;

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

        //(3) Move all samples to observation area
        TrajectorySequence trajectory3a=robot.drive.trajectorySequenceBuilder(specimenScorePose1)
                .lineToLinearHeading(samplePushPath1a)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(samplePushPath1b.getX(),samplePushPath1b.getY()),samplePushPath1bPathAngle) //Position behind 1st sample
                .setReversed(false)
                .build();

        TrajectorySequence trajectory3b=robot.drive.trajectorySequenceBuilder(samplePushPath1b)
                .lineToLinearHeading(samplePushPath2)
                .build();

        TrajectorySequence trajectory3c=robot.drive.trajectorySequenceBuilder(samplePushPath2)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(samplePushPath3.getX(),samplePushPath3.getY()),samplePushPath3PathAngle) //Move around submersible
                .setReversed(false)
                .build();

        TrajectorySequence trajectory3d=robot.drive.trajectorySequenceBuilder(samplePushPath3)
                .lineToLinearHeading(samplePushPath4)
                .build();
/*
        TrajectorySequence trajectory3e=robot.drive.trajectorySequenceBuilder(samplePushPath4)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(samplePushPath5.getX(),samplePushPath5.getY()),samplePushPath5PathAngle) //Move around submersible
                .setReversed(false)
                .build();

        TrajectorySequence trajectory3f=robot.drive.trajectorySequenceBuilder(samplePushPath5)
                .lineToLinearHeading(samplePushPath6)
                .build();
*/

        //===== CYCLE 1 =====
        //(4) Move to floor specimen location, lowering arm to ready position
        TrajectorySequence trajectory4=robot.drive.trajectorySequenceBuilder(samplePushPath4)
               // .addDisplacementMarker(() -> {
               //     CommandScheduler.getInstance().schedule(new ArmPositionCommandGroup(robot.armAngle,1.0,robot.armWinch,1.0,robot.wrist,1.0,robot.claw,1.0,6));
                //})
                .lineToLinearHeading(floorGrabPoseFirst)
                .build();

        //(5) Attempt specimen grab
        TrajectorySequence trajectory5=robot.drive.trajectorySequenceBuilder(floorGrabPoseFirst)
                .waitSeconds(delayGrabAttempt)
                .build();

        //(6) Move to submersible, raising arm to high chamber ready level
        TrajectorySequence trajectory6=robot.drive.trajectorySequenceBuilder(floorGrabPoseFirst)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new ArmPositionCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw,4));
                })
                .lineToLinearHeading(specimenScorePose2)
                //.splineTo(new Vector2d(specimenSecondScoreWaypoint.getX(),specimenSecondScoreWaypoint.getY()),specimenSecondScoreWaypoint.getHeading()) //Move around submersible
                //.splineTo(new Vector2d(specimenScorePose2.getX(),specimenScorePose2.getY()),specimenScorePose2.getHeading()) //Move around submersible leg
                .waitSeconds(delayScoreReady)
                .build();

        //(7) Attempt specimen hang
        TrajectorySequence trajectory7=robot.drive.trajectorySequenceBuilder(specimenScorePose2)
                .waitSeconds(delayScoreAttempt)
                .build();

        //===== CYCLE 2 =====
        //(8) Move to floor specimen location, lowering arm to ready position
        TrajectorySequence trajectory8=robot.drive.trajectorySequenceBuilder(specimenScorePose2)
                //.addDisplacementMarker(() -> {
                //    CommandScheduler.getInstance().schedule(new ArmPositionCommandGroup(robot.armAngle,1.0,robot.armWinch,1.0,robot.wrist,1.0,robot.claw,1.0,6));
                //})
                .lineToLinearHeading(floorGrabPose)
                .build();

        //(9) Attempt specimen grab
        TrajectorySequence trajectory9=robot.drive.trajectorySequenceBuilder(floorGrabPose)
                .waitSeconds(delayGrabAttempt)
                .build();

        //(10) Move to submersible, raising arm to high chamber ready level
        TrajectorySequence trajectory10=robot.drive.trajectorySequenceBuilder(floorGrabPose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new ArmPositionCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw,4));
                })
                .lineToLinearHeading(specimenScorePose3)
                .waitSeconds(delayScoreReady)
                .build();

        //(11) Attempt specimen hang
        TrajectorySequence trajectory11=robot.drive.trajectorySequenceBuilder(specimenScorePose3)
                .waitSeconds(delayScoreAttempt)
                .build();

        //===== CYCLE 3 =====
        //(12) Move to floor specimen location, lowering arm to ready position
        TrajectorySequence trajectory12=robot.drive.trajectorySequenceBuilder(specimenScorePose3)
                //.addDisplacementMarker(() -> {
                //    CommandScheduler.getInstance().schedule(new ArmPositionCommandGroup(robot.armAngle,1.0,robot.armWinch,1.0,robot.wrist,1.0,robot.claw,1.0,6));
                //})
                .lineToLinearHeading(floorGrabPose)
                .build();

        //(13) Attempt specimen grab
        TrajectorySequence trajectory13=robot.drive.trajectorySequenceBuilder(floorGrabPose)
                .waitSeconds(delayGrabAttempt)
                .build();

        //(14) Move to submersible, raising arm to high chamber ready level
        TrajectorySequence trajectory14=robot.drive.trajectorySequenceBuilder(floorGrabPose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new ArmPositionCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw,4));
                })
                .lineToLinearHeading(specimenScorePose4)
                .waitSeconds(delayScoreReady)
                .build();

        //(15) Attempt specimen hang
        TrajectorySequence trajectory15=robot.drive.trajectorySequenceBuilder(specimenScorePose4)
                .waitSeconds(delayScoreAttempt)
                .build();

        //===== CYCLE 4 =====
        /*
        //(16) Move to floor specimen location, lowering arm to ready position
        TrajectorySequence trajectory16=robot.drive.trajectorySequenceBuilder(specimenScorePose4)
                //.addDisplacementMarker(() -> {
                //    CommandScheduler.getInstance().schedule(new ArmPositionCommandGroup(robot.armAngle,1.0,robot.armWinch,1.0,robot.wrist,1.0,robot.claw,1.0,6));
                //})
                .lineToLinearHeading(floorGrabPose)
                .build();

        //(17) Attempt specimen grab
        TrajectorySequence trajectory17=robot.drive.trajectorySequenceBuilder(floorGrabPose)
                .waitSeconds(delayGrabAttempt)
                .build();

        //(18) Move to submersible, raising arm to high chamber ready level
        TrajectorySequence trajectory18=robot.drive.trajectorySequenceBuilder(floorGrabPose)
                .addDisplacementMarker(() -> {
                    CommandScheduler.getInstance().schedule(new ArmPositionCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw,4));
                })
                .lineToLinearHeading(specimenScorePose5)
                .waitSeconds(delayScoreReady)
                .build();

        //(19) Attempt specimen hang
        TrajectorySequence trajectory19=robot.drive.trajectorySequenceBuilder(specimenScorePose5)
                .waitSeconds(delayScoreAttempt)
                .build();
        */
        //===== PARK =====

        //(20) Park
        TrajectorySequence trajectory20=robot.drive.trajectorySequenceBuilder(specimenScorePose4)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(preParkPose.getX(),preParkPose.getY()),preParkPose.getHeading())
                .splineToConstantHeading(new Vector2d(parkPose.getX(),parkPose.getY()),parkPose.getHeading())
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
                //Score 1st Specimen
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
                        currentState= State.TRAJ_3a;
                        robot.drive.followTrajectorySequenceAsync(trajectory3a);
                    }
                    break;

                //Move all samples to observation area
                case TRAJ_3a:
                    if(!robot.drive.isBusy()) {
                        currentState= State.TRAJ_3b;
                        robot.drive.followTrajectorySequenceAsync(trajectory3b);
                    }
                    break;
                case TRAJ_3b:
                    if(!robot.drive.isBusy()) {
                        currentState= State.TRAJ_3c;
                        robot.drive.followTrajectorySequenceAsync(trajectory3c);
                    }
                    break;
                case TRAJ_3c:
                    if(!robot.drive.isBusy()) {
                        currentState= State.TRAJ_3d;
                        robot.drive.followTrajectorySequenceAsync(trajectory3d);
                    }
                    break;
                case TRAJ_3d:
                    if(!robot.drive.isBusy()) {
                        currentState= State.TRAJ_4;
                        robot.drive.followTrajectorySequenceAsync(trajectory4);
                    }
                    break;
                /*
                case TRAJ_3e:
                    if(!robot.drive.isBusy()) {
                        currentState= State.TRAJ_3f;
                        robot.drive.followTrajectorySequenceAsync(trajectory3f);
                    }
                    break;
                case TRAJ_3f:
                    if(!robot.drive.isBusy()) {
                        currentState= State.TRAJ_4;
                        robot.drive.followTrajectorySequenceAsync(trajectory4);
                    }
                    break;
                */

                //Cycle 2
                case TRAJ_4:
                    if(!actionFlag) {
                        CommandScheduler.getInstance().schedule(new ArmPositionCommandGroup(robot.armAngle,0.0,robot.armWinch,0.4,robot.wrist,0.4,robot.claw,0.0,6));
                        actionFlag=true;
                    }

                    if(!robot.drive.isBusy()) {
                        actionFlag=false;
                        currentState= State.TRAJ_5;
                        robot.drive.followTrajectorySequenceAsync(trajectory5);
                    }
                    break;

                case TRAJ_5:
                    if(!actionFlag) {
                        CommandScheduler.getInstance().schedule(new AutonSpecimenFloorGrabCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                        actionFlag=true;
                    }

                    if(!robot.drive.isBusy()) {
                        actionFlag=false;
                        currentState= State.TRAJ_6;
                        robot.drive.followTrajectorySequenceAsync(trajectory6);
                    }
                    break;

                case TRAJ_6:
                    if(!robot.drive.isBusy()) {
                        currentState= State.TRAJ_7;
                        robot.drive.followTrajectorySequenceAsync(trajectory7);
                    }
                    break;

                case TRAJ_7:
                    if(!actionFlag) {
                        CommandScheduler.getInstance().schedule(new AutonSpecimenHangCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                        actionFlag=true;
                    }

                    if(!robot.drive.isBusy()) {
                        actionFlag=false;
                        currentState= State.TRAJ_8;
                        robot.drive.followTrajectorySequenceAsync(trajectory8);
                    }
                    break;

                //Cycle 3
                case TRAJ_8:
                    if(!actionFlag) {
                        CommandScheduler.getInstance().schedule(new ArmPositionCommandGroup(robot.armAngle,1.0,robot.armWinch,1.0,robot.wrist,1.0,robot.claw,1.0,6));
                        actionFlag=true;
                    }

                    if(!robot.drive.isBusy()) {
                        actionFlag=false;
                        currentState= State.TRAJ_9;
                        robot.drive.followTrajectorySequenceAsync(trajectory9);
                    }
                    break;

                case TRAJ_9:
                    if(!actionFlag) {
                        CommandScheduler.getInstance().schedule(new AutonSpecimenFloorGrabCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                        actionFlag=true;
                    }

                    if(!robot.drive.isBusy()) {
                        actionFlag=false;
                        currentState= State.TRAJ_10;
                        robot.drive.followTrajectorySequenceAsync(trajectory10);
                    }
                    break;
                case TRAJ_10:
                    if(!robot.drive.isBusy()) {
                        currentState= State.TRAJ_11;
                        robot.drive.followTrajectorySequenceAsync(trajectory11);
                    }
                    break;
                case TRAJ_11:
                    if(!actionFlag) {
                        CommandScheduler.getInstance().schedule(new AutonSpecimenHangCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                        actionFlag=true;
                    }

                    if(!robot.drive.isBusy()) {
                        actionFlag=false;
                        currentState= State.TRAJ_12;
                        robot.drive.followTrajectorySequenceAsync(trajectory12);
                    }
                    break;

                //Cycle 4
                case TRAJ_12:
                    if(!actionFlag) {
                        CommandScheduler.getInstance().schedule(new ArmPositionCommandGroup(robot.armAngle,1.0,robot.armWinch,1.0,robot.wrist,1.0,robot.claw,1.0,6));
                        actionFlag=true;
                    }

                    if(!robot.drive.isBusy()) {
                        actionFlag=false;
                        currentState= State.TRAJ_13;
                        robot.drive.followTrajectorySequenceAsync(trajectory13);
                    }
                    break;

                case TRAJ_13:
                    if(!actionFlag) {
                        CommandScheduler.getInstance().schedule(new AutonSpecimenFloorGrabCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                        actionFlag=true;
                    }

                    if(!robot.drive.isBusy()) {
                        actionFlag=false;
                        currentState= State.TRAJ_14;
                        robot.drive.followTrajectorySequenceAsync(trajectory14);
                    }
                    break;
                case TRAJ_14:
                    if(!robot.drive.isBusy()) {
                        currentState= State.TRAJ_15;
                        robot.drive.followTrajectorySequenceAsync(trajectory15);
                    }
                    break;
                case TRAJ_15:
                    if(!actionFlag) {
                        CommandScheduler.getInstance().schedule(new AutonSpecimenHangCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                        actionFlag=true;
                    }

                    if(!robot.drive.isBusy()) {
                        actionFlag=false;
                        currentState= State.TRAJ_16;
                        robot.drive.followTrajectorySequenceAsync(trajectory20);
                    }
                    break;
/*
                //Cycle 5
                case TRAJ_16:
                    if(!actionFlag) {
                        CommandScheduler.getInstance().schedule(new ArmPositionCommandGroup(robot.armAngle,1.0,robot.armWinch,1.0,robot.wrist,1.0,robot.claw,1.0,6));
                        actionFlag=true;
                    }

                    if(!robot.drive.isBusy()) {
                        actionFlag=false;
                        currentState= State.TRAJ_17;
                        robot.drive.followTrajectorySequenceAsync(trajectory17);
                    }
                    break;

                case TRAJ_17:
                    if(!actionFlag) {
                        CommandScheduler.getInstance().schedule(new AutonSpecimenFloorGrabCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                        actionFlag=true;
                    }

                    if(!robot.drive.isBusy()) {
                        actionFlag=false;
                        currentState= State.TRAJ_18;
                        robot.drive.followTrajectorySequenceAsync(trajectory18);
                    }
                    break;
                case TRAJ_18:
                    if(!robot.drive.isBusy()) {
                        currentState= State.TRAJ_19;
                        robot.drive.followTrajectorySequenceAsync(trajectory19);
                    }
                    break;
                case TRAJ_19:
                    if(!actionFlag) {
                        CommandScheduler.getInstance().schedule(new AutonSpecimenHangCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw));
                        actionFlag=true;
                    }

                    if(!robot.drive.isBusy()) {
                        actionFlag=false;
                        currentState= State.TRAJ_20;
                        robot.drive.followTrajectorySequenceAsync(trajectory20);
                    }
                    break;
*/
                //Park
                case TRAJ_20:
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
