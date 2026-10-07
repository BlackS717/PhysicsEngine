package physics;

import math.Vector2;

public class CollisionDetector {
    public static Collision detectCollision(Collider collider1, Collider collider2) {

        if(collider1 instanceof CircleCollider && collider2 instanceof CircleCollider) {
            return detectCircleCollision((CircleCollider) collider1, (CircleCollider) collider2);
        }
        else if (collider1 instanceof CircleCollider && collider2 instanceof BoxCollider) {
            return detectCircleBoxCollision((CircleCollider) collider1, (BoxCollider)collider2);
        }
        else if (collider1 instanceof BoxCollider && collider2 instanceof CircleCollider) {
            return detectCircleBoxCollision((CircleCollider) collider2, (BoxCollider) collider1);
        }


        return null;
    }


    private static Collision detectCircleCollision(CircleCollider circle1, CircleCollider circle2) {
        Vector2 distanceVector = circle2.getCenter().sub(circle1.getCenter());

        float distance = (float) Math.sqrt(
            distanceVector.getX() * distanceVector.getX() + distanceVector.getY() * distanceVector.getY()
        );

        boolean colliding =  distance <= (circle1.getRadius() + circle2.getRadius());

        // create the collision object if colliding
        if (colliding) {
            float penetrationDepth = calculatePenetration(circle1, circle2);
            Vector2 normal = calculateNormal(circle1, circle2);
            return new Collision(circle1, circle2, normal, penetrationDepth);
        }

        return  null;
    }

    private static Collision detectCircleBoxCollision(CircleCollider circle, BoxCollider rectangle) {
        // Find the closest point on the rectangle to the circle's center
        Vector2 circleCenter = circle.getCenter();
        float circleRadius = circle.getRadius();


        Vector2 closestPoint = getClosestPoint(rectangle, circleCenter);

        Vector2 distanceVector = circleCenter.sub(closestPoint);

        float distanceSquared = distanceVector.mult(distanceVector).magnitude();

        boolean colliding = distanceSquared <= circleRadius * circleRadius;

        // create the collision object if colliding
        if (colliding) {
            float penetrationDepth = calculatePenetration(circle, rectangle, closestPoint);
            Vector2 normal = calculateNormal(circle, rectangle, closestPoint);
            return new Collision(circle, rectangle, normal, penetrationDepth);
        }

        return null;
    }

    private static Vector2 getClosestPoint(BoxCollider rectangle, Vector2 circleCenter) {
        Vector2 rectCenter = rectangle.getCenter();

        float rectHalfWidth = rectangle.getWidth() / 2;
        float rectHalfHeight = rectangle.getHeight() / 2;

        float xMin = rectCenter.getX() - rectHalfWidth;
        float xMax = rectCenter.getX() + rectHalfWidth;

        float yMin = rectCenter.getY() - rectHalfHeight;
        float yMax = rectCenter.getY() + rectHalfHeight;

        float closestX = Math.clamp(circleCenter.getX(), xMin, xMax);
        float closestY = Math.clamp(circleCenter.getY(), yMin, yMax);

        return new Vector2(closestX, closestY);
    }

        // penetration depth calculation methods

    private static float calculatePenetration(CircleCollider circle1, CircleCollider circle2) {
        Vector2 distanceVector = circle2.getCenter().sub(circle1.getCenter());
        float distance = distanceVector.magnitude();
        return (circle1.getRadius() + circle2.getRadius()) - distance;
    }

    private static float calculatePenetration(CircleCollider circle, BoxCollider rectangle, Vector2 closestPoint) {
        Vector2 distanceVector = circle.getCenter().sub(closestPoint);
        float distance = distanceVector.magnitude();

        // if the circle's center is inside the rectangle, we need to calculate the distance to the closest edge of the rectangle
        if (rectangle.contains(circle.getCenter())) {
            float dEdge = getDEdge(circle, rectangle);

            return circle.getRadius() + dEdge;
        }

        return circle.getRadius() - distance;
    }

    // normal vector calculation methods
     private static Vector2 calculateNormal(CircleCollider circle1, CircleCollider circle2) {
        Vector2 distanceVector = circle2.getCenter().sub(circle1.getCenter());
        return distanceVector.normalize();
    }

    private static Vector2 calculateNormal(CircleCollider circle, BoxCollider rectangle, Vector2 closestPoint) {
        Vector2 distanceVector = circle.getCenter().sub(closestPoint);

        // if the circle's center is inside the rectangle,
        // we need to calculate the normal vector based on the closest edge of the rectangle
        if (rectangle.contains(circle.getCenter())) {
            float[] distances = getDistanceToEdge(circle, rectangle);
            float dLeft = distances[0];
            float dRight = distances[1];
            float dTop = distances[2];
            float dBottom = distances[3];

            if (dLeft < dRight && dLeft < dTop && dLeft < dBottom) {
                distanceVector = new Vector2(-1, 0);
            } else if (dRight < dLeft && dRight < dTop && dRight < dBottom) {
                distanceVector = new Vector2(1, 0);
            } else if (dTop < dLeft && dTop < dRight && dTop < dBottom) {
                distanceVector = new Vector2(0, 1);
            } else {
                distanceVector = new Vector2(0, -1);
            }
        }


        return distanceVector.normalize();
    }

    private static float[] getDistanceToEdge(CircleCollider circle, BoxCollider rectangle) {
        float left = rectangle.getCenter().getX() - rectangle.getWidth() / 2;
        float right = rectangle.getCenter().getX() + rectangle.getWidth() / 2;
        float top = rectangle.getCenter().getY() + rectangle.getHeight() / 2;
        float bottom = rectangle.getCenter().getY() - rectangle.getHeight() / 2;

        float dLeft = Math.abs(left - circle.getCenter().getX());
        float dRight = Math.abs(right - circle.getCenter().getX());
        float dTop = Math.abs(top - circle.getCenter().getY());
        float dBottom = Math.abs(bottom - circle.getCenter().getY());

        return new float[]{dLeft, dRight, dTop, dBottom};
    }

    private static float getDEdge(CircleCollider circle, BoxCollider rectangle) {
        float[] distances = getDistanceToEdge(circle, rectangle);
        float dLeft = distances[0];
        float dRight = distances[1];
        float dTop = distances[2];
        float dBottom = distances[3];

        return Math.min(
                Math.min(
                        dLeft,
                        dRight
                ),
                Math.min(
                        dTop,
                        dBottom
                )
        );
    }

}
