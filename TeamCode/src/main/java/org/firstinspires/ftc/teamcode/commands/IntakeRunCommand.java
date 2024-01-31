package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.concurrent.TimeUnit;

public class IntakeRunCommand extends CommandBase {
   private final IntakeSubsystem intake;
   private double power;
   private double time;
   private ElapsedTime timer;

   public IntakeRunCommand(IntakeSubsystem intake, double power, double time){
       this.intake = intake;
       this.power = power;
       this.time = time;
   }

   @Override
    public void execute(){
       if (timer == null) {
           timer = new ElapsedTime();
           intake.setPower(this.power);
       }

   }

   @Override
    public boolean isFinished() {
       boolean value = false;

       if (timer.time(TimeUnit.SECONDS) >= this.time) {
           intake.setPower(0);
           value = true;
       }

       return value;
   }
}
