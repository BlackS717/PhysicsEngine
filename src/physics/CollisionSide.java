package physics;

import math.Vector2;

/**
 * One side of a Collision, as the solver sees it. It answers the questions the solver has about "the thing that
 * was hit at this point": how fast is it moving there, how hard is it to push there, and what happens to it when
 * an impulse lands there.
 *
 * - BodySide: an ordinary body (linear and angular velocity, lever arm from its centre).
 * - SegmentSide: the middle box of a Segment. A point at fraction s along the segment (0 = end A, 1 = end B) moves
 *   with (1-s) of end A and s of end B, and an impulse landing there is shared between the two ends the same way.
 */
abstract class CollisionSide {

    static CollisionSide of(Body body) {
        return body instanceof SegmentBody ? new SegmentSide((SegmentBody) body) : new BodySide(body);
    }

    /** Velocity of the contact point on this side. */
    abstract Vector2 velocityAt(Vector2 point);

    /** How much this side's contact point speeds up per unit of impulse along `direction` (1 / effective mass). */
    abstract float inverseMassAlong(Vector2 point, Vector2 direction);

    /** Applies an impulse at the contact point. */
    abstract void applyImpulse(Vector2 impulse, Vector2 point);

    /** 0 if this side can't be moved at all. */
    abstract float inverseMass();

    /** Like inverseMassAlong, but for pushing positions apart (spin is ignored). */
    abstract float positionalInverseMass(Vector2 point);

    /** Moves this side by (displacement x its share of the inverse mass) at the contact point. */
    abstract void shiftPosition(Vector2 displacement, Vector2 point);

    /** Snaps barely-moving bodies to rest. */
    abstract void settleIfSlow();

    abstract float restitution();

    abstract float staticFriction();

    abstract float dynamicFriction();

    private static float cross(Vector2 a, Vector2 b) {
        return a.getX() * b.getY() - a.getY() * b.getX();
    }

    // ---------------------------------------------------------------- ordinary body

    private static final class BodySide extends CollisionSide {
        private final Body body;

        BodySide(Body body) {
            this.body = body;
        }

        private Vector2 arm(Vector2 point) {
            return point.sub(body.getTransform().getPosition());
        }

        @Override
        Vector2 velocityAt(Vector2 point) {
            Vector2 r = arm(point);
            float omega = body.getAngularVelocity();
            return body.getVelocity().add(new Vector2(-omega * r.getY(), omega * r.getX()));
        }

        @Override
        float inverseMassAlong(Vector2 point, Vector2 direction) {
            float armCrossDirection = cross(arm(point), direction);
            return body.getInverseMass() + armCrossDirection * armCrossDirection * body.getInverseInertia();
        }

        @Override
        void applyImpulse(Vector2 impulse, Vector2 point) {
            body.setVelocity(body.getVelocity().add(impulse.mult(body.getInverseMass())));
            body.setAngularVelocity(body.getAngularVelocity() + body.getInverseInertia() * cross(arm(point), impulse));
        }

        @Override
        float inverseMass() {
            return body.getInverseMass();
        }

        @Override
        float positionalInverseMass(Vector2 point) {
            return body.getInverseMass();
        }

        @Override
        void shiftPosition(Vector2 displacement, Vector2 point) {
            body.getTransform().setPosition(body.getTransform().getPosition().add(displacement.mult(body.getInverseMass())));
        }

        @Override
        void settleIfSlow() {
            body.settleIfSlow();
        }

        @Override
        float restitution() {
            return body.getRestitution();
        }

        @Override
        float staticFriction() {
            return body.getStaticFriction();
        }

        @Override
        float dynamicFriction() {
            return body.getDynamicFriction();
        }
    }

    // ---------------------------------------------------------------- middle box of a segment

    private static final class SegmentSide extends CollisionSide {
        private final SegmentBody middle;
        private final Body endA;
        private final Body endB;

        SegmentSide(SegmentBody middle) {
            this.middle = middle;
            this.endA = middle.getEndA();
            this.endB = middle.getEndB();
        }

        /** Where along the segment the point is: 0 at end A, 1 at end B. */
        private float fraction(Vector2 point) {
            Vector2 a = endA.getTransform().getPosition();
            Vector2 axis = endB.getTransform().getPosition().sub(a);

            float lengthSquared = axis.dotProduct(axis);
            if (lengthSquared < 1e-12f) return 0.5f;

            float s = point.sub(a).dotProduct(axis) / lengthSquared;
            return Math.max(0f, Math.min(1f, s));
        }

        @Override
        Vector2 velocityAt(Vector2 point) {
            float s = fraction(point);
            return endA.getVelocity().mult(1 - s).add(endB.getVelocity().mult(s));
        }

        @Override
        float inverseMassAlong(Vector2 point, Vector2 direction) {
            return positionalInverseMass(point);
        }

        @Override
        void applyImpulse(Vector2 impulse, Vector2 point) {
            float s = fraction(point);

            endA.setVelocity(endA.getVelocity().add(impulse.mult((1 - s) * endA.getInverseMass())));
            endB.setVelocity(endB.getVelocity().add(impulse.mult(s * endB.getInverseMass())));
        }

        @Override
        float inverseMass() {
            return endA.getInverseMass() + endB.getInverseMass();
        }

        @Override
        float positionalInverseMass(Vector2 point) {
            float s = fraction(point);
            return (1 - s) * (1 - s) * endA.getInverseMass() + s * s * endB.getInverseMass();
        }

        @Override
        void shiftPosition(Vector2 displacement, Vector2 point) {
            float s = fraction(point);

            endA.getTransform().setPosition(endA.getTransform().getPosition()
                    .add(displacement.mult((1 - s) * endA.getInverseMass())));
            endB.getTransform().setPosition(endB.getTransform().getPosition()
                    .add(displacement.mult(s * endB.getInverseMass())));
        }

        @Override
        void settleIfSlow() {
            // a segment held up by tension can be slow without being at rest: its end bodies settle on their own
        }

        @Override
        float restitution() {
            return middle.getRestitution();
        }

        @Override
        float staticFriction() {
            return middle.getStaticFriction();
        }

        @Override
        float dynamicFriction() {
            return middle.getDynamicFriction();
        }
    }
}