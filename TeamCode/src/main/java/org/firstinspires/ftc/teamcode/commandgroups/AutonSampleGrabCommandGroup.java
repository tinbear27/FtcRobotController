package org.firstinspires.ftc.teamcode.commandgroups;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.ArmAngleCommand;
import org.firstinspires.ftc.teamcode.commands.ClawCommand;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ArmAngleSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmWinchSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class AutonSampleGrabCommandGroup extends SequentialCommandGroup {
    private final int positionNumber = 23; //Sample grab position

    public AutonSampleGrabCommandGroup(ArmAngleSubsystem armAngle, ArmWinchSubsystem armWinch, WristSubsystem wrist, ClawSubsystem claw) {

        addCommands(
            new ArmPositionCommandGroup(armAngle,armWinch,wrist,null,positionNumber),

            //Close claw
            new ClawCommand(claw, Constants.Arm.Claw.CLAW_POSITIONS[positionNumber]),
            new WaitCommand(300),

            //Return to travel position -- DO NOT OPEN CLAW
            new ArmPositionCommandGroup(armAngle,armWinch,wrist,null,1)
        );

        addRequirements(armAngle,armWinch,wrist,claw);
    }
}
