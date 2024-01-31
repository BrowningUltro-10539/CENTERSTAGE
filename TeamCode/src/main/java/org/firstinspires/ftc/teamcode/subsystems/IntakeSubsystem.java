package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IntakeSubsystem extends SubsystemBase {
    private final DcMotor intakeMotor;

    public static double intakeSpeed = 1.0;
    public static double pushOutSpeed = -1.0;
    private boolean isAuto;

    private final ElapsedTime timer;
    private final ElapsedTime voltageTimer;
    private final VoltageSensor voltageSensor;
    private double voltage;

    public enum IntakeState{INTAKE, PUSH_OUT, PAUSE}

    public IntakeSubsystem(HardwareMap hardwareMap, boolean isAuto){
        this.intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        this.timer = new ElapsedTime();
        this.voltageTimer = new ElapsedTime();

        timer.reset();
        voltageTimer.reset();

        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
        this.voltage = voltageSensor.getVoltage();

        this.isAuto = isAuto;
    }
    public void update(IntakeState state){
        if (!isAuto)
            switch(state) {
                case INTAKE:
                    intakeMotor.setPower(intakeSpeed);
                    break;
                case PUSH_OUT:
                    intakeMotor.setPower(pushOutSpeed);
                    break;
                case PAUSE:
                    intakeMotor.setPower(0);
            }
    }

    public void setPower(double power){
        intakeMotor.setPower(power);
    }
    public void read(){}
    public void write(){}
    public void loop(){}

}