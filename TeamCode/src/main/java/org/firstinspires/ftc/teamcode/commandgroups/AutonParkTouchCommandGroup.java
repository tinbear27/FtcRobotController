package org.firstinspires.ftc.teamcode.commandgroups;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.ArmAngleCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmAngleSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmWinchSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class AutonParkTouchCommandGroup extends SequentialCommandGroup {
    private final int positionNumber = 24; //Sample grab position

    public AutonParkTouchCommandGroup(ArmAngleSubsystem armAngle, ArmWinchSubsystem armWinch, WristSubsystem wrist, ClawSubsystem claw) {
        //Move arm angle vertical and all other to touch position
        addCommands(
                new ParallelCommandGroup(
                        new ArmPositionCommandGroup(null,0.0,armWinch,0.5,wrist,0.0,claw,0.0,positionNumber),
                        new ArmAngleCommand(armAngle, Constants.Arm.Angle.ANGLE_VERTICAL)
                )
        );

        //Angle
        addCommands(new ArmAngleCommand(armAngle, Constants.Arm.Angle.ANGLE_POSITIONS[24]));

        addRequirements(armAngle,armWinch,wrist,claw);
    }
}

