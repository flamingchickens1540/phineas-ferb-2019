package org.team1540.robot2019.vision;

import java.util.Optional;
import org.team1540.robot2019.Hardware;

public class HasHatchTracker {

    private boolean hasReleasedSinceHadHatch = false;
    private Optional<Boolean> hasHatch = Optional.of(false);

    public void periodic() {
        Optional<Boolean> hatchGrabbed = Hardware.limelight.isHatchGrabbed();
        if (hatchGrabbed.isPresent()) {
            if (hatchGrabbed.get()) { // If vision says we have a hatch, we probably do have a hatch (rare false positives)
                hasHatch = Optional.of(true);
            } else if  // If vision says we don't have a hatch,
            (hasHatch.orElse(false) // and we defiantly did have a hatch
                    && hasReleasedSinceHadHatch()) { // and the hatch mechanism has released since we last had a hatch, we probably don't have a hatch (common false negatives)
                hasHatch = Optional.of(false);
                hasReleasedSinceHadHatch = false;
            }
        }
    }

    private boolean hasReleasedSinceHadHatch() { // TODO: This whole class isn't finished
        return hasReleasedSinceHadHatch;
    }

    public void hatchMechWasReleased() {
        if (hasHatch.orElse(false)) { // If we definitely do have a hatch
            hasReleasedSinceHadHatch = true;
        }
    }

    public void hatchMechWasGrabbed() {
        if (!hasHatch.orElse(false)) { // If we definitely do not have a hatch

        }
    }

    public boolean hasHatch() { // Is usually correct except when the hatch has just been grabbed
        return hasHatch.get();
    }
}
