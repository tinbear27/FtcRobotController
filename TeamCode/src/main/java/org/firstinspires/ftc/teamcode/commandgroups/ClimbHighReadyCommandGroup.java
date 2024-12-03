package org.firstinspires.ftc.teamcode.commandgroups;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.ClimbersCommand;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ArmAngleSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmWinchSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClimbersSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class ClimbHighReadyCommandGroup extends SequentialCommandGroup {

    public ClimbHighReadyCommandGroup(ClimbersSubsystem climbers, ArmAngleSubsystem armAngle, ArmWinchSubsystem armWinch, WristSubsystem wrist) {
/*
        addCommands(

                new ParallelCommandGroup(
                    new ClimbersCommand(climbers,Constants.Climbers.CLIMBER_HIGH_READY_POSITION),
                    new SequentialCommandGroup(
                        new WaitCommand(400),
                        new ArmPositionCommandGroup(armAngle,0.2,armWinch,0.6,wrist,0.0,null,0.0,18).withTimeout(1500)
                    )
                ),
                new InstantCommand(() -> RobotHardware.getInstance().setClimbState(Constants.Climbers.CLIMB_STATE.HIGHRUNG_READY))
        );

        addRequirements(climbers,armAngle,armWinch,wrist);
*/
    }
}