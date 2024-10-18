package org.firstinspires.ftc.teamcode.commandgroups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.WristCommand;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class WristDelayCommandGroup  extends SequentialCommandGroup {
    public WristDelayCommandGroup(WristSubsystem wrist, double wristTarget, double delay) {
        addCommands(new WaitCommand((long)(delay*1000)));
        addCommands(new WristCommand(wrist,wristTarget));

        addRequirements(wrist);
    }
}
