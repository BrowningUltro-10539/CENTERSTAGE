package org.firstinspires.ftc.teamcode.subsystems.old;


import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftSubsystemDouble extends SubsystemBase {
    public final MotorEx lift1;
    public final MotorEx lift2;
    private final PIDController controller;

    private MotionProfile profile;
    public MotionState currentState;

    public double P = 0;
    public double I = 0;
    public double D = 0;
    public double kG = 0;

    public double currentLiftPosition;
    public double targetLiftPosition;
    public double liftPower;
    //change ticks of slide motors(check specks)
    private final double slidesTickPerInch = 2 * Math.PI * 0.701771654 / 537.7;

    private boolean isAuto = false;


    public LiftSubsystemDouble(HardwareMap hardwareMap, boolean isAuto){
        this.lift1 = new MotorEx(hardwareMap, "liftOne");
        this.lift2 = new MotorEx(hardwareMap, "liftTwo");
        this.controller = new PIDController(P, I, D);


        this.isAuto = isAuto;
    }

    public void read(){
        currentLiftPosition = lift1.encoder.getPosition() * slidesTickPerInch ;

    }

    public void loop() {
        liftPower = controller.calculate(currentLiftPosition, targetLiftPosition) + kG;
    }

    public double getLiftPos(){
        return currentLiftPosition;
    }

    public void write(){
        lift1.set(-liftPower);
        lift2.set(liftPower);
    }

    public void setTargetLiftPosition(double value){
        targetLiftPosition = value;
    }

    public void newLiftProfile(double targetLiftPosition, double max_v, double max_a){
        profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(getLiftPos(), 0), new MotionState(targetLiftPosition, 0), max_v, max_a);
    }
}
