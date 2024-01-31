package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeRunCommand extends CommandBase {
   private final IntakeSubsystem intake;
   private double power;
   private double time;

   public IntakeRunCommand(IntakeSubsystem intake, double power){
       this.intake = intake;
       this.power = power;
   }

   @Override
    public void execute(){
       intake.setPower(this.power);
   }
}
