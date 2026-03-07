package frc.robot;

public class EdgeCounter {

    public enum EdgeType {
        RISING, FALLING, BOTH
    }

    private boolean initialState = true;

    private int count;
    private boolean previousState;
    private final EdgeType type;

    /**
     * Creates a new counter, with the initial state set to true.
     */
    public EdgeCounter(EdgeType type) {
        this(type, true);
    }

    /**
     * Creates a new counter, with the initial state set to true.
     */
    public EdgeCounter(EdgeType type, boolean initialState) {
        this.reset();
        this.type = type;
        this.initialState = initialState;
    }

    /**
     * Evaluate a new input, and increment the counter if transitioning from {@code false} to {@code true}.
     * @param currentState The new state to evaluate
     */
    public void update(boolean currentState) {
        // increment if transition matches type
        if (
                (this.type == EdgeType.RISING && !this.previousState && currentState) ||
                (this.type == EdgeType.FALLING && this.previousState && !currentState) ||
                (this.type == EdgeType.BOTH && this.previousState != currentState)
        ) {
            this.count++;
        }
        this.previousState = currentState;
    }

    /**
     * Get the amount of detected rising edges.
     * @return The current rising edge count
     */
    public int getCount() {
        return this.count;
    }

    /**
     * Resets the current counter to zero, and resets the previous state to the default.
     */
    public void reset() {
        this.count = 0;
        this.previousState = initialState;
    }
}