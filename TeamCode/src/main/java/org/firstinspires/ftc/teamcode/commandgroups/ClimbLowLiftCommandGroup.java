package org.firstinspires.ftc.teamcode.commandgroups;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.ClimbersCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommand;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ClimbersSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class ClimbLowLiftCommandGroup extends SequentialCommandGroup {

    public ClimbLowLiftCommandGroup(LiftSubsystem lift, ClimbersSubsystem climbers) {
/*
        addCommands(

                new ClimbersCommand(climbers,Constants.Climbers.CLIMBER_LOW_ENGAGE_POSITION),
                new ParallelCommandGroup(
                    new LiftCommand(lift, Constants.Lift.LIFT_MIN),
                    new SequentialCommandGroup(
                            new WaitCommand(3400),
                            new ClimbersCommand(climbers,Constants.Climbers.CLIMBER_START_POSITION)
                    )
                ),
                new InstantCommand(() -> RobotHardware.getInstance().setClimbState(Constants.Climbers.CLIMB_STATE.LOWRUNG_LIFT))
        );

        addRequirements(lift,climbers);
*/
    }
}