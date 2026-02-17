package com.team9470.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.team9470.subsystems.shooter.Shooter;
import com.team9470.subsystems.swerve.Swerve;
import edu.wpi.first.wpilibj2.command.Commands;

public class Autos {
  private final AutoFactory m_autoFactory;
  private final Shooter m_shooter;

  public Autos(Swerve swerve, Shooter shooter) {
    m_shooter = shooter;
    m_autoFactory = swerve.createAutoFactory((sample, isStart) -> {
    });
  }

  public AutoRoutine doNothing() {
    AutoRoutine routine = m_autoFactory.newRoutine("DoNothing");
    routine.active().onTrue(Commands.none());
    return routine;
  }

  public AutoRoutine newPath() {
    AutoRoutine routine = m_autoFactory.newRoutine("NewPath");
    AutoTrajectory newPath = routine.trajectory("NewPath");

    routine.active().onTrue(
        newPath.resetOdometry()
            .andThen(newPath.cmd()));
    return routine;
  }
}
