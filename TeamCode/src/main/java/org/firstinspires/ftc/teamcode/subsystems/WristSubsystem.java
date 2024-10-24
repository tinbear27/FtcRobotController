package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.Arm.Wrist;

public class WristSubsystem extends SubsystemBase {
    private final Servo wristServo;
    private double currentPosition;
    private final Telemetry thisTelemetry;

    public WristSubsystem(final HardwareMap hMap, Telemetry telemetry) {
        wristServo = hMap.get(Servo.class, Wrist.WRIST_ID);
        this.thisTelemetry=telemetry;
    }

    public void setPositionByIndex(int indexNum) {
        setPosition(Wrist.WRIST_POSITIONS[indexNum]);
    }

    public void setPosition(double position) {
        this.currentPosition=position;
        wristServo.setPosition(position);
    }

    public double getPosition() {
        return wristServo.getPosition();
    }
}
