package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LiftSubsystem extends SubsystemBase {

    public final MotorEx lift;

    public double P = 0.3;
    public double I = 0;
    public double D = 0;
    public double kG = 0.1;


    private MotionProfile profile;
    public MotionState currentState;

    // --
    private final ElapsedTime timer;
    private final ElapsedTime voltageTimer;
    private final PIDController controller;
    private final VoltageSensor voltageSensor;


    private double voltage;
    private double liftPosition;
    private double targetLiftPosition = 0.0;
    private double liftPower = 0.0;
    // ----

    //change ticks of slide motors(check specks)
    private final double slidesTickPerInch = 2 * Math.PI * 0.701771654 / 537.7;

    private boolean isAuto = false;


    public LiftSubsystem(HardwareMap hardwareMap, boolean isAuto){
        this.lift = new MotorEx(hardwareMap, "lift");

        this.lift.setInverted(true);

        // -- It's better to use motion profiling which allows us to control the acceleration and velocity of the slide.
        this.timer = new ElapsedTime();
        timer.reset();

        this.profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(1,0), new MotionState(0,0), 30, 25);

        this.voltageTimer = new ElapsedTime();
        voltageTimer.reset();

        this.controller = new PIDController(P, I, D);
        controller.setPID(P, I, D);

        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
        this.voltage = voltageSensor.getVoltage();
        // --

        this.isAuto = isAuto;
    }

    public void loop() {
        // We update the voltage periodically to ensure that the power being provided to the slide is proportional to the voltage of the robot.
        if (voltageTimer.seconds() > 5) {
            voltage = voltageSensor.getVoltage();
            voltageTimer.reset();
        }

        /**
         * With motion profiling, we're trying to power the slides such that it goes up at some specific velocity and acceleration.
         * We have some goal target position (i.e. 15 inches (from 0 inches)) and we are trying to go at a velocity of 30 in/sec.
         * Motion profiling will create a function that depicts the behavior you are trying to model.
         * Instead of telling the PID controller to provide some power to the desired goal position (15 inches),
         * the MotionProfile will give a smaller targetPosition that we input into the PID controller
         * such that it's power is maintained to result in the lift to go up at 30 in/sec.
         **/
        currentState = profile.get(timer.time());
        if(currentState.getV() != 0) {
            targetLiftPosition = currentState.getX();
        }

        liftPower = (controller.calculate(liftPosition, targetLiftPosition) + kG) / voltage * 14;
    }

    public double getLiftPos(){
        return liftPosition;
    }


    public void read(){ liftPosition = lift.encoder.getPosition() * slidesTickPerInch ;}
    public void write(){
        lift.set(liftPower);
    }

    public void setTargetLiftPosition(double value){
        targetLiftPosition = value;
    }

    public void resetTimer(){
        timer.reset();
    }

    public void newLiftProfile(double targetLiftPosition, double max_v, double max_a){
        profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(getLiftPos(), 0), new MotionState(targetLiftPosition, 0), max_v, max_a);
        resetTimer();
    }
}
