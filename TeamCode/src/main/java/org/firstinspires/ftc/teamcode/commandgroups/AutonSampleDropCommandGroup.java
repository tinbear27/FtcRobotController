package org.firstinspires.ftc.teamcode.commandgroups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.ClawCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmAngleSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmWinchSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class AutonSampleDropCommandGroup extends SequentialCommandGroup {

    public AutonSampleDropCommandGroup(ArmAngleSubsystem armAngle, ArmWinchSubsystem armWinch, WristSubsystem wrist, ClawSubsystem claw) {

        addCommands(
            new ClawCommand(claw, Constants.Arm.Claw.CLAW_OPEN),
            new WaitCommand(200),
            new ArmPositionCommandGroup(armAngle,0.0,armWinch,0.2,wrist,0.3,null,0.0,1)
        );

        addRequirements(armAngle,armWinch,wrist,claw);
    }
}
