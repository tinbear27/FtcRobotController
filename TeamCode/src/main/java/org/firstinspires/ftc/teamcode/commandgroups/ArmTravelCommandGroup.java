package org.firstinspires.ftc.teamcode.commandgroups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.WristCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmAngleSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmWinchSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class ArmTravelCommandGroup extends SequentialCommandGroup {

    public ArmTravelCommandGroup(ArmAngleSubsystem armAngle, ArmWinchSubsystem armWinch, WristSubsystem wrist, ClawSubsystem claw) {
        //Move wrist to horizontal position first, then start winch retraction before all other movements
        addCommands(
            new WristCommand(wrist, Constants.Arm.Wrist.WRIST_HORIZ_POS),
            new ArmPositionCommandGroup(armAngle, 0.3, armWinch,0.0, wrist,0.3, null,0.0,1)
        );

        addRequirements(armAngle,armWinch,wrist,claw);
    }
}
