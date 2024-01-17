package swerve.params;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;

public class DriveRampingConfigs {
    public final OpenLoopRampsConfigs openLoopConfigs;
    public final ClosedLoopRampsConfigs closedLoopConfigs;

    public DriveRampingConfigs() {
        this.openLoopConfigs = (new OpenLoopRampsConfigs()).withDutyCycleOpenLoopRampPeriod(0.25);
        this.closedLoopConfigs = new ClosedLoopRampsConfigs();
    }

    public DriveRampingConfigs(OpenLoopRampsConfigs openLoopRampsConfigs,
            ClosedLoopRampsConfigs closedLoopRampsConfigs) {
        this.openLoopConfigs = openLoopRampsConfigs;
        this.closedLoopConfigs = closedLoopRampsConfigs;
    }
}
