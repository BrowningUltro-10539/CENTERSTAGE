package org.firstinspires.ftc.teamcode.subsystems.old;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HangerSubsystem extends SubsystemBase {

    public final MotorEx hangerMotor;

    private final PIDController hangerController;

    private MotionProfile profile;
    public MotionState currentState;


    public static double P = 0.3;
    public static double I = 0;
    public static double D = 0;
    public static double kG = 0.0;


    private boolean isAuto;

    private final double HANGER_TICKS_PER_INCH = 2 * Math.PI * 0.375/751.8;
    public double hangerPower = 0.0;
    public double hangerTargetPosition = 0.0;
    public double currentHangerPosition;



    public HangerSubsystem(HardwareMap hardwaremap, boolean isAuto){
        this.hangerMotor = new MotorEx(hardwaremap, "hangerMotor");

        this.hangerController = new PIDController(P,I,D);

        this.isAuto = isAuto;
    }

    public void read(){
        currentHangerPosition = hangerMotor.encoder.getPosition() + HANGER_TICKS_PER_INCH;
    }
    public void loop(){
        hangerPower = hangerController.calculate(currentHangerPosition, hangerTargetPosition) + kG;
    }
    public void write(){
        hangerMotor.set(hangerPower);
    }

    public void setHangerTargetPosition(double value){
        hangerTargetPosition = value;
    }

    public double getHangerPos(){
        return currentHangerPosition;
    }
    public void newHangerProfile(double targetLiftPosition, double max_v, double max_a){
        profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(getHangerPos(), 0), new MotionState(targetLiftPosition, 0), max_v, max_a);

    }

}
