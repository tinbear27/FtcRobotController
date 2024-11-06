package org.firstinspires.ftc.teamcode.commandgroups;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.ArmAngleCommand;
import org.firstinspires.ftc.teamcode.commands.ClawCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmAngleSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmWinchSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class AutonSpecimenFloorGrabCommandGroup extends SequentialCommandGroup {
    private final int positionNumber = 7; //Basket drop position

    public AutonSpecimenFloorGrabCommandGroup(ArmAngleSubsystem armAngle, ArmWinchSubsystem armWinch, WristSubsystem wrist, ClawSubsystem claw) {

        addCommands(
            //Lower arm to grab position
            new ArmPositionCommandGroup(armAngle,0.0,1000,armWinch,0.0,0,wrist,0.0,claw,0.2,positionNumber),

            //Return to ready position -- DO NOT OPEN CLAW
            new ArmAngleCommand(armAngle,Constants.Arm.Angle.ANGLE_POSITIONS[6]+Constants.Arm.Angle.ANGLE_EXTRA_TICKS_POST_GRAB)
        );

        addRequirements(armAngle,armWinch,wrist,claw);
    }
}
