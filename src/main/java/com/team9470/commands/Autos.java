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

  public AutoRoutine trenchCenterRightBlue() {
    AutoRoutine routine = m_autoFactory.newRoutine("trenchCenterRightBlue");
    AutoTrajectory trenchRightBlue = routine.trajectory("trenchRightBlue");

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

  public AutoRoutine trenchCenterLeftBlue() {
    AutoRoutine routine = m_autoFactory.newRoutine("trenchCenterLeftBlue");
    AutoTrajectory trenchLeftBlue = routine.trajectory("trenchLeftBlue");

    routine.active().onTrue(
        trenchLeftBlue.resetOdometry()
            .andThen(trenchLeftBlue.cmd())
            .andThen(Superstructure.getInstance().aimAndShootCommand()));
    
    trenchLeftBlue.atTime("IntakeDown")
        .onTrue(new InstantCommand(() -> Superstructure.getInstance().getIntake().setDeployed(true)));
    trenchLeftBlue.atTime("IntakeUp")
        .onTrue(new InstantCommand(() -> Superstructure.getInstance().getIntake().setDeployed(false)));
    return routine;
  }

  public AutoRoutine bumpCenterRightBlue() {
    AutoRoutine routine = m_autoFactory.newRoutine("bumpCenterRightBlue");
    AutoTrajectory bumpRightBlue = routine.trajectory("bumpRightBlue");

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
}
