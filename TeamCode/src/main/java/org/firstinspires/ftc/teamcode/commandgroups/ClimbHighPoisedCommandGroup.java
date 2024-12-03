package org.firstinspires.ftc.teamcode.commandgroups;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.ClimbersCommand;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ClimbersSubsystem;

public class ClimbHighPoisedCommandGroup extends SequentialCommandGroup {

    public ClimbHighPoisedCommandGroup(ClimbersSubsystem climbers) {
/*
        addCommands(
            new ClimbersCommand(climbers, Constants.Climbers.CLIMBER_HIGH_RETRY_POSITION),
            new InstantCommand(() -> RobotHardware.getInstance().setClimbState(Constants.Climbers.CLIMB_STATE.HIGHRUNG_POISED))
        );

        addRequirements(climbers);
        */

    }
}
