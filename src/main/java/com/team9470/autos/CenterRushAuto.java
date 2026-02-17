package com.team9470.autos;

import com.team9470.subsystems.Superstructure;
import com.team9470.subsystems.swerve.Swerve;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CenterRushAuto extends SequentialCommandGroup {

        public CenterRushAuto() {
                Swerve swerve = Swerve.getInstance();
                Superstructure superstructure = Superstructure.getInstance();

                var autoFactory = swerve.createAutoFactory();

                var pathCommand = autoFactory.trajectoryCmd("centerauto");

                addCommands(
                        pathCommand.deadlineWith(superstructure.intakeCommand()),
                        superstructure.aimAndShootCommand());
        }
}
