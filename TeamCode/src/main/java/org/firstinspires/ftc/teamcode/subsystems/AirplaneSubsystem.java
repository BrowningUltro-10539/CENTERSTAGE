package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class AirplaneSubsystem extends SubsystemBase {

    private final Servo airplaneServo;

    public static double open = 1.0;
    public static double closed = 0.0;

    public enum airplaneServoState{OPEN, CLOSED}

    public AirplaneSubsystem(HardwareMap hardwareMap) {
        airplaneServo = hardwareMap.get(Servo.class, "portC4");

    }

    public void read(){

    }
    public void loop(){
    }
    public void write(){
    }

    public void update(airplaneServoState state){
        switch(state){
            case OPEN:
                setAirplaneServoState(1.0);
                break;
            case CLOSED:
                setAirplaneServoState(0.0);
                break;
        }
    }
    public void setAirplaneServoState(double position){
        airplaneServo.setPosition(position);
    }

}
