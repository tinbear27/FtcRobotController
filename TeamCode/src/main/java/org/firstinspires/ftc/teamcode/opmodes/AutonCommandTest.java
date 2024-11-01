package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.commands.TrajectoryCommand;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@Disabled
public class AutonCommandTest extends CommandOpMode {
    private final RobotHardware robot = RobotHardware.getInstance();

    private TrajectoryCommand trajectoryFollower;

    @Override
    public void initialize() {
        robot.init(hardwareMap,telemetry);

        Trajectory traj1 = robot.driveSubsystem.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();
        trajectoryFollower = new TrajectoryCommand(robot.driveSubsystem, traj1);

        schedule(new WaitUntilCommand(this::isStarted).andThen(
                trajectoryFollower.andThen(new WaitCommand(2000),
                        new TrajectoryCommand(robot.driveSubsystem,
                                robot.driveSubsystem.trajectoryBuilder(traj1.end(), true)
                                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                                        .build()
                        ))
        ));
    }

}
