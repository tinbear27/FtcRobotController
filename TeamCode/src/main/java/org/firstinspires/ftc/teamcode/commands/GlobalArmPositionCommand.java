package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class GlobalArmPositionCommand  extends InstantCommand {

    //Update Current Position Only
    public GlobalArmPositionCommand(int position) {
        Globals.ARM_POSITION=position;
    }
}
