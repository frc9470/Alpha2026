package com.team9470.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.team9470.choreo.ChoreoTraj;

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

  public AutoRoutine trenchRightBlue() {
    AutoRoutine routine = m_autoFactory.newRoutine("trenchRightBlue");
    AutoTrajectory trenchRightBlue = ChoreoTraj.trenchRightBlue.asAutoTraj(routine);

    routine.active().onTrue(
        trenchRightBlue.resetOdometry()
            .andThen(new InstantCommand(() -> Superstructure.getInstance().getIntake().setDeployed(true)))
            .andThen(trenchRightBlue.cmd())
            .andThen(Superstructure.getInstance().aimAndShootCommand()));
    return routine;
  }

  public AutoRoutine bumpRightBlue() {
    AutoRoutine routine = m_autoFactory.newRoutine("bumpRightBlue");
    AutoTrajectory bumpRightBlue = ChoreoTraj.bumpRightBlue.asAutoTraj(routine);

    routine.active().onTrue(
        bumpRightBlue.resetOdometry()
            .andThen(bumpRightBlue.cmd())
            .andThen(Superstructure.getInstance().aimAndShootCommand()));

    bumpRightBlue.atTime("intakeDown")
        .onTrue(new InstantCommand(() -> Superstructure.getInstance().getIntake().setDeployed(true)));
    return routine;
  }
}
