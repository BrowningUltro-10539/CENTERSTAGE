package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HangerSubsystem extends CommandBase {

    private final Servo hangerServo;
    private final DcMotor hangerMotor;


    public enum ServoState {UP, DOWN}
    public enum PullingState{PULL, PUSH, WAIT}

    public HangerSubsystem(HardwareMap hardwareMap){
        this.hangerMotor = hardwareMap.get(DcMotor.class, "hangerMotor");
        this.hangerServo = hardwareMap.get(Servo.class, "portC3");
    }


    public void update(ServoState state){
        switch (state){
            case UP:
                setHangerServo(0);
                break;
            case DOWN:
                setHangerServo(0.37);
                break;
        }
    }

    public void update(PullingState state){
        switch (state){
            case PULL:
                setPower(-1);
                break;
            case PUSH:
                setPower(1);
                break;
            case WAIT:
                setPower(0);
        }
    }


    public void setHangerServo(double pos){
        hangerServo.setPosition(pos);
    }

    public void setPower(double power){
        hangerMotor.setPower(power);
    }
}
