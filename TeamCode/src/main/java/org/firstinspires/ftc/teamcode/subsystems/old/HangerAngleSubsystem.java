package org.firstinspires.ftc.teamcode.subsystems.old;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HangerAngleSubsystem extends SubsystemBase {
    public final MotorEx hangerAngleMotor;

    private final PIDController hangerAngleController;

    private MotionProfile profile;
    public MotionState currentState;


    public static double P = 0.3;
    public static double I = 0;
    public static double D = 0;
    public static double kG = 0;


    private boolean isAuto;

    private final double HANGER_ANGLE_TICKS_PER_DEGREE = 12.8571429/145.1;
    public double hangerPower = 0.0;
    public double hangerTargetPosition = 0.0;
    public double currentHangerPosition;



    public HangerAngleSubsystem(HardwareMap hardwaremap, boolean isAuto){
        this.hangerAngleMotor = new MotorEx(hardwaremap, "hangerMotor");

        this.hangerAngleController = new PIDController(P,I,D);

        this.isAuto = isAuto;
    }

    public void read(){
        currentHangerPosition = hangerAngleMotor.encoder.getPosition() + HANGER_ANGLE_TICKS_PER_DEGREE;
    }
    public void loop(){
        hangerPower = hangerAngleController.calculate(currentHangerPosition, hangerTargetPosition) + kG;
    }
    public void write(){
        hangerAngleMotor.set(hangerPower);
    }

    public void setHangerAngleTargetPosition(double value){
        hangerTargetPosition = value;
    }

    public double getHangerAnglePos(){
        return currentHangerPosition;
    }
    public void newHangerAngleProfile(double targetLiftPosition, double max_v, double max_a){
        profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(getHangerAnglePos(), 0), new MotionState(targetLiftPosition, 0), max_v, max_a);

    }

}
