package org.firstinspires.ftc.teamcode.commandgroups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.ClawCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmAngleSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmWinchSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class AutonBasketDropCommandGroup extends SequentialCommandGroup {
    private final int positionNumber = 10; //Basket drop position

    public AutonBasketDropCommandGroup(ArmAngleSubsystem armAngle, ArmWinchSubsystem armWinch, WristSubsystem wrist, ClawSubsystem claw) {

        addCommands(
                new ClawCommand(claw, Constants.Arm.Claw.CLAW_OPEN),
                new WaitCommand(800),
                new ArmPositionCommandGroup(armAngle,0.0,armWinch,0.5,wrist,0.3,null,0.0,1)
        );

        addRequirements(armAngle,armWinch,wrist,claw);
    }
}
