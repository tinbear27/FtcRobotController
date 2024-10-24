package org.firstinspires.ftc.teamcode.commandgroups;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.ClimbersCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommand;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ClimbersSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class ClimbLowReadyCommandGroup extends SequentialCommandGroup {

    public ClimbLowReadyCommandGroup(LiftSubsystem lift, ClimbersSubsystem climbers) {
        addCommands(
                new ParallelCommandGroup(
                    new LiftCommand(lift,Constants.Lift.LIFT_MAX),
                    new ClimbersCommand(climbers,Constants.Climbers.CLIMBER_LOW_READY_POSITION)
                ),
                new InstantCommand(() -> RobotHardware.getInstance().setClimbState(Constants.Climbers.CLIMB_STATE.LOWRUNG_READY))
        );

        addRequirements(lift,climbers);
    }
}