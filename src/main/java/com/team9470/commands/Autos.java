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
            .andThen(trenchRightBlue.cmd())
            .andThen(Superstructure.getInstance().aimAndShootCommand()));

    trenchRightBlue.atTime("IntakeDown")
        .onTrue(new InstantCommand(() -> Superstructure.getInstance().getIntake().setDeployed(true)));
    trenchRightBlue.atTime("IntakeUp")
        .onTrue(new InstantCommand(() -> Superstructure.getInstance().getIntake().setDeployed(false)));
    return routine;
  }

  public AutoRoutine bumpRightBlue() {
    AutoRoutine routine = m_autoFactory.newRoutine("bumpRightBlue");
    AutoTrajectory bumpRightBlue = ChoreoTraj.bumpRightBlue.asAutoTraj(routine);

    routine.active().onTrue(
        bumpRightBlue.resetOdometry()
            .andThen(bumpRightBlue.cmd())
            .andThen(Superstructure.getInstance().aimAndShootCommand()));

    bumpRightBlue.atTime("IntakeDown")
        .onTrue(new InstantCommand(() -> Superstructure.getInstance().getIntake().setDeployed(true)));
    bumpRightBlue.atTime("IntakeUp")
        .onTrue(new InstantCommand(() -> Superstructure.getInstance().getIntake().setDeployed(false)));
    return routine;
  }

  public AutoRoutine bumpCenterLeftBlue() {
    AutoRoutine routine = m_autoFactory.newRoutine("bumpCenterLeftBlue");
    AutoTrajectory bumpLeftBlue = routine.trajectory("bumpLeftBlue");

    routine.active().onTrue(
        bumpLeftBlue.resetOdometry()
            .andThen(bumpLeftBlue.cmd())
            .andThen(Superstructure.getInstance().aimAndShootCommand()));

    bumpLeftBlue.atTime("IntakeDown")
        .onTrue(new InstantCommand(() -> Superstructure.getInstance().getIntake().setDeployed(true)));
    bumpLeftBlue.atTime("IntakeUp")
        .onTrue(new InstantCommand(() -> Superstructure.getInstance().getIntake().setDeployed(false)));
    return routine;
  }

  public AutoRoutine depotOutpostBlue() {
    AutoRoutine routine = m_autoFactory.newRoutine("depotOutpostBlue");
    AutoTrajectory depotOutpostBlue = routine.trajectory("depotOutpostBlue");

    routine.active().onTrue(
        depotOutpostBlue.resetOdometry()
            .andThen(depotOutpostBlue.cmd())
            .andThen(Superstructure.getInstance().aimAndShootCommand()));

    depotOutpostBlue.atTime("IntakeDown")
        .onTrue(new InstantCommand(() -> Superstructure.getInstance().getIntake().setDeployed(true)));
    return routine;
  }

  public AutoRoutine figure8() {
    AutoRoutine routine = m_autoFactory.newRoutine("figure8");
    AutoTrajectory figure8 = routine.trajectory("testing");

    routine.active().onTrue(
        figure8.resetOdometry()
            .andThen(figure8.cmd()));
    return routine;
  }
}
