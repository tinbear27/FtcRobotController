package org.firstinspires.ftc.teamcode.commandgroups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.ClawCommand;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

public class ClawDelayCommandGroup  extends SequentialCommandGroup {
    public ClawDelayCommandGroup(ClawSubsystem claw, double clawTarget, double delay) {
        addCommands(new WaitCommand((long)(delay*1000)));
        addCommands(new ClawCommand(claw,clawTarget));

        addRequirements(claw);
    }
}
