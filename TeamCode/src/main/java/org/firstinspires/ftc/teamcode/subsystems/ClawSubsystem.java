package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.Arm.Claw;

public class ClawSubsystem extends SubsystemBase {
    private final Servo clawServo;
    private final Telemetry thisTelemetry;

    public ClawSubsystem(final HardwareMap hMap, Telemetry telemetry) {
        clawServo = hMap.get(Servo.class, Claw.CLAW_ID);
        this.thisTelemetry=telemetry;
    }

    public void grab() {
        setPosition(Claw.CLAW_CLOSED);
    }

    public void release() {
        setPosition(Claw.CLAW_OPEN);
    }

    public void setPositionByIndex(int indexNum) {
        setPosition(Claw.CLAW_POSITIONS[indexNum]);
    }

    public void setPosition(double position) {
        clawServo.setPosition(position);
    }

    public double getPosition() {
        return clawServo.getPosition();
    }
}
