// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.constants.ConstantsArm;


public class TrapezoidArmSubsystem extends TrapezoidProfileSubsystem {
  
  private ArmFeedforward armFeedforward;
  private double acceleration;
  
  private VictorSP armMotor_1, armMotor_2;
  private SpeedControllerGroup armMotorsGroup;

  private AnalogGyro gyro;

  public TrapezoidArmSubsystem(int ID_1, int ID_2, AnalogGyro gyro) {

    super(new TrapezoidProfile.Constraints(ConstantsArm.MAX_VELOCITY, ConstantsArm.MAX_ACCELERATION), 
                                           ConstantsArm.INITIAL_POSITION);

    this.gyro = gyro;
    this.armMotor_1 = new VictorSP(ID_1);
    this.armMotor_2 = new VictorSP(ID_2);
    this.armMotorsGroup = new SpeedControllerGroup(armMotor_1, armMotor_2);
    this.armFeedforward = new ArmFeedforward(ConstantsArm.kS, 
                                             ConstantsArm.kCOS,
                                             ConstantsArm.kV, 
                                             ConstantsArm.kA);
  }

  public SpeedControllerGroup getArmMotorsGroup() {
    return armMotorsGroup;
  }

  public AnalogGyro getGyro() {
    return gyro;
  }

  public void setAcceleration(double acceleration) {
    this.acceleration = acceleration;
  }

  @Override
  public void useState(TrapezoidProfile.State state) {
    double feedforward = armFeedforward.calculate(state.position, state.velocity, acceleration); 
    this.armMotorsGroup.setVoltage(feedforward);
  }
}