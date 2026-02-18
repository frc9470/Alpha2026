package com.team9470.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

import com.team9470.subsystems.Superstructure;
import com.team9470.subsystems.swerve.Swerve;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Autos {
  private final AutoFactory m_autoFactory;

  public Autos(Swerve swerve) {
    m_autoFactory = swerve.createAutoFactory();
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

  public AutoRoutine trenchRightAuto() {
    AutoRoutine routine = m_autoFactory.newRoutine("trenchRightAuto");
    AutoTrajectory trenchRightAuto = routine.trajectory("trenchRightAuto");

    routine.active().onTrue(
        trenchRightAuto.resetOdometry()
            .andThen(new InstantCommand(() -> Superstructure.getInstance().getIntake().setDeployed(true)))
            .andThen(trenchRightAuto.cmd())
            .andThen(Superstructure.getInstance().aimAndShootCommand()));
    return routine;
  }
}
