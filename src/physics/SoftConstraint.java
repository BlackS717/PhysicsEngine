package physics;

/**
 * The spring behaviour shared by Segment (length) and SegmentJoint (bend angle).
 *
 * - rigidity  0..1: how strongly the shape resists being deformed. 1 = fully rigid (a hard constraint).
 *                   Below 1 the constraint becomes a spring whose frequency goes from MIN_FREQUENCY (0) up to
 *                   MAX_FREQUENCY (just below 1), spread evenly on a log scale.
 * - elasticity 0..1: how much of the stored energy comes back as motion. 1 = almost undamped (rings and flings
 *                   things), 0 = critically damped (returns to shape slowly without overshooting).
 *
 * The spring is solved implicitly (a "soft constraint": the spring's stiffness and damping are folded into the
 * constraint's effective mass), so it stays stable however stiff it is. The frequency is fixed by rigidity, so a
 * heavier structure takes more force to deform by the same amount.
 *
 * Limit: the implicit solver damps a spring by itself, by about frequency x 2 x PI x dt / 2 of the damping ratio.
 * That is already compensated for, but it means a very stiff spring (near MAX_FREQUENCY at 60 fps) can't ring
 * for long however high its elasticity is: with only 4 steps per oscillation there is nothing left to ring with.
 */
final class SoftConstraint {
    static final float MIN_FREQUENCY = 0.25f; // Hz, rigidity 0
    static final float MAX_FREQUENCY = 15f;   // Hz, rigidity -> 1 (keep well below fps / 2 for accurate motion)
    private static final float MIN_DAMPING_RATIO = 0.02f;

    private float rigidity = 1f;
    private float elasticity = 0.8f;

    // per-step solver state, filled in by prepare()
    private float gamma;
    private float bias;
    private float accumulatedImpulse;

    void setRigidity(float rigidity) {
        this.rigidity = Math.max(0f, Math.min(1f, rigidity));
    }

    float getRigidity() {
        return rigidity;
    }

    void setElasticity(float elasticity) {
        this.elasticity = Math.max(0f, Math.min(1f, elasticity));
    }

    float getElasticity() {
        return elasticity;
    }

    boolean isRigid() {
        return rigidity >= 1f;
    }

    /** Spring frequency in Hz (meaningless when rigid). */
    float frequency() {
        return (float) (MIN_FREQUENCY * Math.pow(MAX_FREQUENCY / MIN_FREQUENCY, rigidity));
    }

    /** 1 = critically damped, small = ringing. */
    float dampingRatio() {
        return Math.max(MIN_DAMPING_RATIO, 1f - elasticity);
    }

    /** Spring constant for a given effective mass: force per metre (or torque per radian). Infinite when rigid. */
    float stiffness(float effectiveMass) {
        if (isRigid()) return Float.POSITIVE_INFINITY;
        float omega = (float) (2 * Math.PI * frequency());
        return effectiveMass * omega * omega;
    }

    /**
     * Call once per step before the solver iterations.
     *
     * @param effectiveMass 1 / (inverse mass of the constraint)
     * @param error         how far the constraint is from its rest value (stretch, or angle difference)
     * @param dt            time step in seconds
     */
    void prepare(float effectiveMass, float error, float dt) {
        accumulatedImpulse = 0f;

        if (isRigid() || effectiveMass <= 0f || dt <= 0f) {
            gamma = 0f;
            bias = 0f;
            return;
        }

        float omega = (float) (2 * Math.PI * frequency());

        // the implicit solver damps the spring by itself (about omega * dt / 2 of the damping ratio, a lot for
        // stiff springs at 60 fps), so only add what is still missing to reach the damping that elasticity asks for
        float solverDamping = 0.5f * omega * dt;
        float addedDamping = Math.max(0f, dampingRatio() - solverDamping);

        float damping = 2f * effectiveMass * addedDamping * omega;
        float stiffness = effectiveMass * omega * omega;

        float denominator = dt * (damping + dt * stiffness);
        gamma = denominator > 0f ? 1f / denominator : 0f;
        bias = error * dt * stiffness * gamma;
    }

    /**
     * One solver iteration.
     *
     * @param constraintVelocity how fast the constrained quantity is changing right now
     * @param inverseMass        the constraint's inverse effective mass
     * @return the impulse to apply
     */
    float solve(float constraintVelocity, float inverseMass) {
        float denominator = inverseMass + gamma;
        if (denominator <= 0f) return 0f;

        float impulse = -(constraintVelocity + bias + gamma * accumulatedImpulse) / denominator;
        accumulatedImpulse += impulse;
        return impulse;
    }
}