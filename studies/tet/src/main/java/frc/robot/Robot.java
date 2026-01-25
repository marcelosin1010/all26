// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.System.Logger.Level;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.logging.primitive.PrimitiveLogger;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode;
import org.team100.lib.motor.ctre.Kraken6Motor;
import org.team100.lib.motor.rev.NeoVortexCANSparkMotor;
import org.team100.lib.util.CanId;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private final NeoVortexCANSparkMotor top;
  private final NeoVortexCANSparkMotor bottom;

  private static final LoggerFactory rootLogger = Logging.instance().rootLogger;

  public Robot() {
    m_robotContainer = new RobotContainer();
    LoggerFactory parent = rootLogger.type(this);

    top = new NeoVortexCANSparkMotor(parent.name("Top"), new CanId(1), NeutralMode.BRAKE, MotorPhase.FORWARD, 200,
        Feedforward100.makeNeoVortex(rootLogger), PIDConstants.makeVelocityPID(rootLogger, 0.0002,0.0000001,0.0004));
    bottom = new NeoVortexCANSparkMotor(parent.name("Bottom"), new CanId(2), NeutralMode.BRAKE, MotorPhase.FORWARD, 200,
        Feedforward100.makeNeoVortex(rootLogger), PIDConstants.makeVelocityPID(rootLogger, 0.00005,0.00000001,0.0001));
  }
  @Override
  public void robotPeriodic() {
    Takt.update();
    Cache.refresh();
    top.periodic();
    bottom.periodic();
    CommandScheduler.getInstance().run();
    NetworkTableInstance.getDefault().flush();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    top.setVelocity(Math.PI * -160, 0, 0);
    bottom.setVelocity(Math.PI * 160, 0, 0);
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
