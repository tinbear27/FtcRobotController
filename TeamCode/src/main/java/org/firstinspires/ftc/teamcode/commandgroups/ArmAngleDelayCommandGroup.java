package org.firstinspires.ftc.teamcode.commandgroups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.ArmAngleCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmAngleSubsystem;

public class ArmAngleDelayCommandGroup extends SequentialCommandGroup {

    public ArmAngleDelayCommandGroup(ArmAngleSubsystem armAngle, int armAngleTarget, double delay) {
        addCommands(new WaitCommand((long)(delay*1000)));
        addCommands(new ArmAngleCommand(armAngle,armAngleTarget));

        addRequirements(armAngle);
    }
}
