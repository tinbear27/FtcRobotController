package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commandgroups.ArmPositionCommandGroup;
import org.firstinspires.ftc.teamcode.commandgroups.AutonSpecimenFloorGrabCommandGroup;
import org.firstinspires.ftc.teamcode.commandgroups.AutonSpecimenHangCommandGroup;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utility.PoseStorage;

@Autonomous(group = "Autonomous",name="Park (only)", preselectTeleOp="Manual Drive")
public class AutonPark extends LinearOpMode {

    enum State {
        TRAJ_1,   // Park
        IDLE      // Prior to start
    }

    State currentState = State.IDLE;

    private final RobotHardware robot = RobotHardware.getInstance();

    //Define our start position (against wall, facing right, back of robot lined up with tile teeth near net zone tape)
    Pose2d startPose = new Pose2d(8.5, -63, Math.toRadians(90));

    //Define all trajectory start/end poses
    Pose2d parkPose=new Pose2d(57, -60, Math.toRadians(90));

    private double loopTime = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();

        robot.init(hardwareMap,telemetry);

        //Set starting pose
        robot.drive.setPoseEstimate(startPose);

        //Define all trajectories

        //(1) Park
        TrajectorySequence trajectory1=robot.drive.trajectorySequenceBuilder(startPose)
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
