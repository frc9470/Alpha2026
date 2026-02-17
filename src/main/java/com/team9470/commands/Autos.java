package com.team9470.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

import com.team9470.subsystems.Superstructure;
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

  public AutoRoutine shoot8() {
    AutoRoutine routine = m_autoFactory.newRoutine("Shoot8");
    AutoTrajectory moveToCenter = routine.trajectory("moveToCenter");

    routine.active().onTrue(
        moveToCenter.resetOdometry()
            .andThen(moveToCenter.cmd())
            .andThen(Superstructure.getInstance().aimAndShootCommand()));

    moveToCenter.atTime("deployIntake").onTrue(Superstructure.getInstance().toggleIntakeCommand());
    return routine;
  }
}
