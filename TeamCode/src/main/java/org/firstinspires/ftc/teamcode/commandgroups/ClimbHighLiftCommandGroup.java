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

public class ClimbHighLiftCommandGroup extends SequentialCommandGroup {

    public ClimbHighLiftCommandGroup(ClimbersSubsystem climbers, ArmAngleSubsystem armAngle, ArmWinchSubsystem armWinch, WristSubsystem wrist) {

        /*
            addCommands(
                    new ParallelCommandGroup(
                        new ClimbersCommand(climbers, Constants.Climbers.CLIMBER_START_POSITION),
                        new SequentialCommandGroup(
                            new WaitCommand(500),
                            new ArmPositionCommandGroup(armAngle,0.7,armWinch,0.0,wrist,0.0,null,0.0,25)
                        )
                    ),
                new InstantCommand(() -> RobotHardware.getInstance().setClimbState(Constants.Climbers.CLIMB_STATE.SUMMIT))
            );

            addRequirements(climbers,armAngle,armWinch,wrist);
*/
    }
}