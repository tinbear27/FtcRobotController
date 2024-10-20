package org.firstinspires.ftc.teamcode.commandgroups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.ArmAngleSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmWinchSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class AutonSampleReadyCommandGroup extends SequentialCommandGroup {
    private final int positionNumber = 22; //Basket drop position

    public AutonSampleReadyCommandGroup(ArmAngleSubsystem armAngle, ArmWinchSubsystem armWinch, WristSubsystem wrist, ClawSubsystem claw) {
        addCommands(new ArmPositionCommandGroup(armAngle,0.0,armWinch,0.0,wrist,0.0,claw,0.500,positionNumber));
        addRequirements(armAngle,armWinch,wrist,claw);
    }
}
