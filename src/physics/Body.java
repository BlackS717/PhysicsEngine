package physics;

import math.Transform;
import math.Vector2;
import rendering.BodyRenderInfo;

public class Body {
    // bodies are circular shape

    private Transform transform;
    private Vector2 velocity;
    private Vector2 acceleration;

    // rotation (radians): angular velocity in rad/s, angular acceleration in rad/s^2
    private float angularVelocity;
    private float angularAcceleration;

    private final Transform initialTransform;
    private final Vector2 initialVelocity;
    private final Vector2 initialAcceleration;
    private final float initialAngularVelocity;
    private final float initialAngularAcceleration;

    // pose at the start of the last integrate() call, used by continuous collision detection
    private Vector2 previousPosition;
    private float previousRotation;

    private Collider collider;

    private float mass;
    private float inverseMass;

    private float restitutionCoefficient = 1.0f; // 1.0f is perfectly elastic, 0.0f is perfectly inelastic

    // 0.0f is frictionless. Static should be >= dynamic: it takes more to start sliding than to keep sliding.
    private float staticFrictionCoefficient = 0.6f;
    private float dynamicFrictionCoefficient = 0.4f;

    // damping (1/s): the fraction of velocity lost per second. Angular damping also acts as rolling resistance.
    private float linearDamping = 0.05f;
    private float angularDamping = 0.5f;

    // a body in contact that moves/spins slower than these is snapped to rest (m/s, rad/s)
    private float linearRestThreshold = 0.05f;
    private float angularRestThreshold = 0.05f;

    private boolean isStatic = false;

    // rendering
    private BodyRenderInfo renderInfo = new BodyRenderInfo();

    public Body() {
        this(new Transform(), 1.0f);
    }

    public Body(Transform transform, float mass) throws IllegalArgumentException {
        if (mass <= 0) {
            throw new IllegalArgumentException("Mass must be positive.");
        }

        this.transform = transform;
        this.velocity = new Vector2(0, 0);
        this.acceleration = new Vector2(0, 0);
        this.angularVelocity = 0f;
        this.angularAcceleration = 0f;
        this.mass = mass;
        this.inverseMass = 1.0f / mass;

        initialTransform = new Transform(transform);
        initialVelocity = new Vector2(velocity);
        initialAcceleration = new Vector2(acceleration);
        initialAngularVelocity = angularVelocity;
        initialAngularAcceleration = angularAcceleration;

        previousPosition = new Vector2(transform.getPosition());
        previousRotation = transform.getRotation();
    }

    public void setCollider(Collider collider) {
        this.collider = collider;
    }

    public Collider getCollider() {
        return this.collider;
    }

    /***
     * dt is the time between each tick, 1 at 1fps and 0.5 at 2fps.
     * to get the value, you divide 1 by the desired fps.
     * */
    public void integrate(float dt) {
        if (isStatic) return;

        previousPosition = new Vector2(transform.getPosition());
        previousRotation = transform.getRotation();

        updatePosition(dt);
        updateRotation(dt);

        updateVelocity(dt);
        updateAngularVelocity(dt);
    }

    private void updatePosition(float dt) {
        this.transform.setPosition(
                this.transform.getPosition().add(
                        this.velocity.mult(dt).add(
                                this.acceleration.mult(dt * dt * 0.5f)
                        )
                )
        );
    }

    private void updateRotation(float dt) {
        this.transform.rotate(angularVelocity * dt + angularAcceleration * dt * dt * 0.5f);
    }

    private void updateVelocity(float dt) {
        Vector2 newVelocity = this.velocity.add(this.acceleration.mult(dt));

        // implicit damping: stable for any dt, never flips the sign of the velocity
        setVelocity(newVelocity.mult(1.0f / (1.0f + linearDamping * dt)));
    }

    private void updateAngularVelocity(float dt) {
        float newAngularVelocity = this.angularVelocity + this.angularAcceleration * dt;

        setAngularVelocity(newAngularVelocity / (1.0f + angularDamping * dt));
    }

    /**
     * Snaps the body to rest if it is barely moving and barely spinning.
     * Called after a contact has been resolved, so a body at the top of a throw is never frozen mid-air.
     */
    public void settleIfSlow() {
        if (isStatic) return;

        if (velocity.magnitude() < linearRestThreshold && Math.abs(angularVelocity) < angularRestThreshold) {
            setVelocity(new Vector2(0, 0));
            setAngularVelocity(0f);
        }
    }

    public void applyForces(Vector2... forces) {
        // reset the acceleration
        resetAcceleration();

        // apply the forces
        Vector2 forcesSum = new Vector2(0, 0);

        for (Vector2 force : forces) {
            forcesSum = forcesSum.add(force);
        }

        // update acceleration
        setAcceleration(forcesSum.div(mass));
    }

    private void resetAcceleration() {
        this.acceleration.setCoordinates(0, 0);
    }

    // Getters and Setters
    public void setVelocity(Vector2 velocity) {
        this.velocity = isStatic ? new Vector2(0, 0) : velocity;
    }

    public Vector2 getVelocity() {
        return this.velocity;
    }

    public void setAcceleration(Vector2 acceleration) {
        this.acceleration = acceleration;
    }

    public Vector2 getAcceleration() {
        return this.acceleration;
    }

    public void setAngularVelocity(float angularVelocity) {
        this.angularVelocity = isStatic ? 0f : angularVelocity;
    }

    public float getAngularVelocity() {
        return this.angularVelocity;
    }

    public void setAngularAcceleration(float angularAcceleration) {
        this.angularAcceleration = angularAcceleration;
    }

    public float getAngularAcceleration() {
        return this.angularAcceleration;
    }

    public float getMass() {
        return this.mass;
    }

    public void setMass(float mass) {
        if (mass <= 0) {
            throw new IllegalArgumentException("Mass must be positive.");
        }

        this.mass = mass;
        this.inverseMass = isStatic ? 0.0f : 1.0f / mass;
    }

    public float getInverseMass() {
        return this.inverseMass;
    }

    /**
     * Inverse moment of inertia, derived from the collider shape and the current mass.
     * Circle: 1/2 m r^2. Box: 1/12 m (w^2 + h^2). Static or collider-less bodies never rotate from impulses (0).
     */
    public float getInverseInertia() {
        if (isStatic || collider == null) return 0.0f;

        float inertia;
        if (collider instanceof CircleCollider) {
            float r = ((CircleCollider) collider).getRadius();
            inertia = 0.5f * mass * r * r;
        } else if (collider instanceof BoxCollider) {
            float w = ((BoxCollider) collider).getWidth();
            float h = ((BoxCollider) collider).getHeight();
            inertia = mass * (w * w + h * h) / 12.0f;
        } else {
            return 0.0f;
        }

        return inertia > 0 ? 1.0f / inertia : 0.0f;
    }

    public void setLinearDamping(float linearDamping) {
        this.linearDamping = Math.max(0f, linearDamping);
    }

    public float getLinearDamping() {
        return this.linearDamping;
    }

    public void setAngularDamping(float angularDamping) {
        this.angularDamping = Math.max(0f, angularDamping);
    }

    public float getAngularDamping() {
        return this.angularDamping;
    }

    public void setRestThresholds(float linearThreshold, float angularThreshold) {
        this.linearRestThreshold = Math.max(0f, linearThreshold);
        this.angularRestThreshold = Math.max(0f, angularThreshold);
    }

    public void setStaticFriction(float staticFriction) {
        this.staticFrictionCoefficient = staticFriction;
    }

    public float getStaticFriction() {
        return this.staticFrictionCoefficient;
    }

    public void setDynamicFriction(float dynamicFriction) {
        this.dynamicFrictionCoefficient = dynamicFriction;
    }

    public float getDynamicFriction() {
        return this.dynamicFrictionCoefficient;
    }

    public void setRestitution(float restitution) {
        this.restitutionCoefficient = restitution;
    }

    public float getRestitution() {
        return this.restitutionCoefficient;
    }

    public Transform getTransform() {
        return transform;
    }

    public BodyRenderInfo getRenderInfo() {
        return this.renderInfo;
    }

    public void reset() {
        this.transform = new Transform(initialTransform);
        this.acceleration = new Vector2(initialAcceleration);
        this.velocity = new Vector2(initialVelocity);
        this.angularVelocity = initialAngularVelocity;
        this.angularAcceleration = initialAngularAcceleration;

        this.previousPosition = new Vector2(this.transform.getPosition());
        this.previousRotation = this.transform.getRotation();
    }

    public boolean isStatic() {
        return this.isStatic;
    }

    /** Position at the start of the last integrate() call (the current position for bodies that never moved). */
    public Vector2 getPreviousPosition() {
        return previousPosition;
    }

    /** Rotation (radians) at the start of the last integrate() call. */
    public float getPreviousRotation() {
        return previousRotation;
    }

    /** Overrides the pose used as "start of the last step" (for bodies that are positioned by other bodies). */
    public void setPreviousPose(Vector2 position, float rotation) {
        this.previousPosition = position;
        this.previousRotation = rotation;
    }

    /**
     * Moves the body back to where it was fraction t (0..1) of the way through the last integrate() step:
     * 0 = where it started, 1 = where it ended up. Used by continuous collision detection.
     */
    public void moveToFractionOfLastStep(float t) {
        Vector2 end = transform.getPosition();
        transform.setPosition(previousPosition.add(end.sub(previousPosition).mult(t)));
        transform.setRotation(previousRotation + (transform.getRotation() - previousRotation) * t);
    }

    public void setStatic(boolean isStatic) {
        this.isStatic = isStatic;
        this.inverseMass = isStatic ? 0.0f : 1.0f / mass;
    }
}