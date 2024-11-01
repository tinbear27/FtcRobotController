package org.firstinspires.ftc.teamcode.commandgroups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.WristCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmAngleSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmWinchSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class ArmTravelAfterGrabCommandGroup  extends SequentialCommandGroup {

    public ArmTravelAfterGrabCommandGroup(ArmAngleSubsystem armAngle, ArmWinchSubsystem armWinch, WristSubsystem wrist) {
        addCommands(
                new WristCommand(wrist, Constants.Arm.Wrist.WRIST_HORIZ_POS),
                new ArmPositionCommandGroup(armAngle,0.45,armWinch,0.0,wrist,0.3,null,0.0,1)
        );

        addRequirements(armAngle,armWinch,wrist);
    }
}
