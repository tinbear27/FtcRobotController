package org.firstinspires.ftc.teamcode.commandgroups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.ArmWinchCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmWinchSubsystem;

public class ArmWinchDelayCommandGroup extends SequentialCommandGroup {
    public ArmWinchDelayCommandGroup(ArmWinchSubsystem armWrist, int armWinchTarget, double delay) {
        addCommands(new WaitCommand((long)(delay*1000)));
        addCommands(new ArmWinchCommand(armWrist,armWinchTarget));

        addRequirements(armWrist);
    }
}
