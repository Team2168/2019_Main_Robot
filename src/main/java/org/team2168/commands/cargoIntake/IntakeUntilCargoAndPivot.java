package org.team2168.commands.cargoIntake;

import org.team2168.commands.auto.Sleep;
import org.team2168.commands.hatchProbePivot.DriveHatchProbePivotWithConstant;
import org.team2168.commands.monkeyBarIntakeWheels.DriveMonkeyBarIntakeWithConstant;
import org.team2168.commands.monkeyBarPivot.interlocks.MoveMonkeyBarToCargoIntakePosition;
import org.team2168.commands.monkeyBarPivot.interlocks.MoveMonkeyBarToSafePositionForScoring;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class IntakeUntilCargoAndPivot extends CommandGroup {

    public IntakeUntilCargoAndPivot() {
		addParallel(new MoveMonkeyBarToCargoIntakePosition());
		addParallel(new DriveMonkeyBarIntakeWithConstant(0.75));//add robot map
		addSequential(new IntakeUntilCargo());

		//we have a cargo
		addParallel(new DriveMonkeyBarIntakeWithConstant(0.0));
		addParallel(new DriveHatchProbePivotWithConstant(-0.75),1.5);
		addSequential(new Sleep(), 0.5);
		addSequential(new MoveMonkeyBarToSafePositionForScoring());
    	
    }
}
 