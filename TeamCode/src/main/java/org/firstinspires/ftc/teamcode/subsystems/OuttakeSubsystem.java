package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeSubsystem extends SubsystemBase {

    private final Servo leftArm;
//    private final Servo rightArm;
    private final Servo bucketStopper;


    public static double ARM_DOWN = 0.15;
    public static double ARM_UP = 1;


    private boolean isAuto;

    public enum ArmState {INTAKE, RELEASE, RETRACT}
    public enum BucketState{OPEN, CLOSED}

    public OuttakeSubsystem(HardwareMap hardwareMap, boolean isAuto){
        this.leftArm = hardwareMap.get(Servo.class, "portC1");
//        this.rightArm = hardwareMap.get(Servo.class, "portC2");
        this.bucketStopper = hardwareMap.get(Servo.class, "portC2");

        this.isAuto = isAuto;
    }

    public void update(ArmState state){
        switch(state){
            case INTAKE:
                setArm(ARM_DOWN);
                break;
            case RELEASE:
                setArm(ARM_UP);
                break;
            case RETRACT:
                setArm(0.7);
                break;
            }

    }

    public void update(BucketState state){
        switch(state){
            case OPEN:
                setBucketStopper(1);
                break;
            case CLOSED:
                setBucketStopper(0);
                break;
        }
    }


    public void read(){}
    public void write(){}
    public void loop(){}

    public void setArm(double pos){
        leftArm.setPosition(pos);
//        rightArm.setPosition(1-pos);
    }

    public void setBucketStopper(double pos){
        bucketStopper.setPosition(pos);
    }
}