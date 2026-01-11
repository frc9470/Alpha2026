package com.team9470.simulation;

import edu.wpi.first.math.geometry.Translation3d;

public class PhysicsSim {

    public static class Projectile {
        public Translation3d position;
        public Translation3d velocity;
        public Translation3d angularVelocity; // rad/s

        // Physics Constants
        private static final double GRAVITY = 9.81; // m/s^2
        private static final double AIR_DENSITY = 1.225; // kg/m^3
        private static final double DRAG_COEFF = 0.5; // Approx for foam ball
        private static final double MASS = 0.27; // kg (~9.5 oz / 270g approx for 2024/2025 notes? Or 2026? Using
                                                 // ~0.27kg generic)
        public static final double RADIUS = 0.07; // m
        private static final double AREA = Math.PI * RADIUS * RADIUS;

        public Projectile(Translation3d position, Translation3d velocity) {
            this(position, velocity, new Translation3d());
        }

        public Projectile(Translation3d position, Translation3d velocity, Translation3d angularVelocity) {
            this.position = position;
            this.velocity = velocity;
            this.angularVelocity = angularVelocity;
        }

        /**
         * Updates the projectile physics state for a single timestep.
         * 
         * @param dt         Time step in seconds.
         * @param goalCenter The center of the target (funnel) for collision checks.
         * @return true if the projectile "scored" (passed through the funnel throat),
         *         false otherwise.
         */
        public boolean update(double dt, Translation3d goalCenter) {
            // Forces
            // 1. Gravity: Fg = m * g (down)
            Translation3d Fg = new Translation3d(0, 0, -MASS * GRAVITY);

            // 2. Drag: Fd = -1/2 * rho * A * Cd * v^2 * v_hat
            double speed = velocity.getNorm();
            Translation3d Fd = new Translation3d();
            if (speed > 1e-6) {
                double dragMag = 0.5 * AIR_DENSITY * AREA * DRAG_COEFF * speed * speed;
                Fd = velocity.div(speed).times(-dragMag);
            }

            // 3. Magnus: Fm = 1/2 * rho * A * r * (w x v)
            // (Simplified Lift Force model)
            Translation3d Fm = new Translation3d();
            if (angularVelocity.getNorm() > 1e-6 && speed > 1e-6) {
                // Cross product: w x v
                // Manual Cross Product since Translation3d might not return Translation3d or
                // might be missing method
                double wx = angularVelocity.getX();
                double wy = angularVelocity.getY();
                double wz = angularVelocity.getZ();
                double vx = velocity.getX();
                double vy = velocity.getY();
                double vz = velocity.getZ();

                double cx = wy * vz - wz * vy;
                double cy = wz * vx - wx * vz;
                double cz = wx * vy - wy * vx;

                Translation3d wCrossV = new Translation3d(cx, cy, cz);

                // Coeff: 0.5 * rho * A * r?
                // Vector form: 0.5 * rho * A * r * (w x v)
                double liftFactor = 0.5 * AIR_DENSITY * AREA * RADIUS;
                Fm = wCrossV.times(liftFactor);
            }

            // Net Force
            Translation3d Fnet = Fg.plus(Fd).plus(Fm);

            // Acceleration: a = F / m
            Translation3d accel = Fnet.div(MASS);

            // Symplectic Euler (or semi-implicit) roughly
            velocity = velocity.plus(accel.times(dt));

            // Old Gravity Application (Removed in favor of Fnet)
            // velocity = velocity.minus(new Translation3d(0, 0, GRAVITY * dt));

            // Field Collision (Floor & Walls)
            // Field Dimensions (approximate from AllianceFlipUtil)
            double fieldLen = 16.54;
            double fieldWidth = 8.05;
            double ballRadius = 0.07;
            double restitution = 0.6;
            double floorFriction = 0.05; // Friction per tick on floor

            Translation3d nextPos = position.plus(velocity.times(dt));

            // Floor Collision
            if (nextPos.getZ() <= ballRadius) {
                // Bounce Z
                if (velocity.getZ() < 0) {
                    velocity = new Translation3d(
                            velocity.getX() * (1.0 - floorFriction), // Friction
                            velocity.getY() * (1.0 - floorFriction),
                            -velocity.getZ() * restitution);
                }
                nextPos = new Translation3d(nextPos.getX(), nextPos.getY(), ballRadius);
            }

            // Wall Collision (X)
            if (nextPos.getX() <= ballRadius || nextPos.getX() >= fieldLen - ballRadius) {
                velocity = new Translation3d(
                        -velocity.getX() * restitution,
                        velocity.getY(),
                        velocity.getZ());
                // Clamp position
                if (nextPos.getX() <= ballRadius)
                    nextPos = new Translation3d(ballRadius, nextPos.getY(), nextPos.getZ());
                if (nextPos.getX() >= fieldLen - ballRadius)
                    nextPos = new Translation3d(fieldLen - ballRadius, nextPos.getY(), nextPos.getZ());
            }

            // Wall Collision (Y)
            if (nextPos.getY() <= ballRadius || nextPos.getY() >= fieldWidth - ballRadius) {
                velocity = new Translation3d(
                        velocity.getX(),
                        -velocity.getY() * restitution,
                        velocity.getZ());
                // Clamp
                if (nextPos.getY() <= ballRadius)
                    nextPos = new Translation3d(nextPos.getX(), ballRadius, nextPos.getZ());
                if (nextPos.getY() >= fieldWidth - ballRadius)
                    nextPos = new Translation3d(nextPos.getX(), fieldWidth - ballRadius, nextPos.getZ());
            }

            // Update Position

            // Check Funnel Collision
            // Goal Logic:
            // Top Hex: R=24" (0.61m), Z=72" (1.83m)
            // Bot Hex: R=13.37" (0.34m), Z=56.5" (1.44m)
            // 6 Planes.

            double topR = 0.6096;
            double botR = 0.3396;
            double topZ = 1.8288;
            double botZ = 1.4351;

            // If we are around the goal Z range
            if (nextPos.getZ() < topZ + 0.5 && nextPos.getZ() > botZ - 0.5) {
                // Check relative to goal center
                Translation3d localPos = nextPos.minus(goalCenter);

                // Check if scored (passed through bottom hex)
                if (nextPos.getZ() < botZ && position.getZ() >= botZ) {
                    // Check if inside bottom hex radius (approximate as circle for simplicity or
                    // check hex)
                    // Hex apothem is R * sqrt(3)/2.
                    // Let's rely on the funnel walls physics to guide it in.
                    // If it hasn't bounced OUT, and is below Z, it's Scored.
                    // Simple radial check:
                    double r = Math.hypot(localPos.getX(), localPos.getY());
                    if (r < botR) {
                        return true; // Scored (removed)
                    }
                }

                // Funnel Wall Collision
                if (nextPos.getZ() >= botZ && nextPos.getZ() <= topZ) {
                    for (int i = 0; i < 6; i++) {
                        double angle = Math.toRadians(60 * i);
                        // Normal to the hex side i points outwards from center?
                        // Face center is at angle.
                        // Let's define plane equation.
                        // Project localPos onto radial vector at angle.
                        // Apothem distance R(z) linearly interpolates.

                        double cosA = Math.cos(angle);
                        double sinA = Math.sin(angle);

                        // Distance of point projected onto apothem direction
                        double radialDist = localPos.getX() * cosA + localPos.getY() * sinA;

                        // Max allowed radial distance at this Z
                        // Apothem at Top: TopR * sqrt(3)/2
                        // Apothem at Bot: BotR * sqrt(3)/2
                        double apoTop = topR * Math.sqrt(3) / 2.0;
                        double apoBot = botR * Math.sqrt(3) / 2.0;
                        double factor = (nextPos.getZ() - botZ) / (topZ - botZ);
                        double maxDist = apoBot + (apoTop - apoBot) * factor;

                        // Check collision (ball radius)

                        // FIX: Ignore collision if we are clearly outside the funnel geometry
                        // This prevents "infinite wall" effect for missed shots.
                        // Tolerance: Ball Diameter
                        if (radialDist > maxDist + ballRadius * 2.0) {
                            continue;
                        }

                        if (radialDist > maxDist - ballRadius) {
                            // Collision!
                            // Reflect velocity.
                            // Normal vector of the plane?
                            // Plane is defined by line at angle.
                            // Slope in Z.
                            // Normal has Z component component because walls are slanted.
                            // Wall slopes OUT. So Normal points IN and DOWN.
                            // Slope m = (apoTop - apoBot) / (topZ - botZ)
                            // Normal 2D (Radial-Z plane): (-slope, 1) normalized?
                            // Wall Vector (dr, dz). Normal (-dz, dr).
                            // dr = apoTop - apoBot. dz = topZ - botZ.
                            double dr = apoTop - apoBot;
                            double dz = topZ - botZ;

                            // Normal in Radial-Z coords:
                            // To point IN (negative radial) and DOWN?
                            // Vector UP face is (dr, dz).
                            // Inward normal is (-dz, dr)?
                            // (-0.39, 0.18). Points In and Up.
                            // Wait, if ball hits wall moving down, and wall slopes out, it should bounce
                            // IN.
                            // Normal (-dz, dr) means Normal_Z is positive (dr is pos).
                            // So Normal points UP.
                            // Yes, wall faces UP and IN.

                            double normRadial2D = -dz;
                            double normZ2D = dr;
                            double len = Math.hypot(normRadial2D, normZ2D);
                            normRadial2D /= len;
                            normZ2D /= len;

                            Translation3d normal = new Translation3d(
                                    normRadial2D * cosA,
                                    normRadial2D * sinA,
                                    normZ2D);

                            // Reflect: V = V - (1+e)*(V.N)*N
                            double vDotN = velocity.getX() * normal.getX() +
                                    velocity.getY() * normal.getY() +
                                    velocity.getZ() * normal.getZ();

                            // Only bounce if moving INTO wall (vDotN < 0)
                            if (vDotN < 0) {
                                velocity = velocity.minus(normal.times((1 + restitution) * vDotN));

                                // Push out of wall
                                double overlap = (radialDist - (maxDist - ballRadius));
                                // Move opposite to normal direction? Or just radial push?
                                // Let's simplify and push radially in.
                                // But real push should be along normal.
                                nextPos = nextPos.plus(normal.times(overlap * 1.1));
                            }
                        }
                    }
                }
            }

            position = nextPos;
            return false;
        }
    }

    /**
     * Resolves collision between two projectiles (Elastic collision).
     */
    public static void resolveCollision(Projectile a, Projectile b) {
        double radius = Projectile.RADIUS;
        double dist = a.position.getDistance(b.position);

        if (dist < radius * 2) {
            // Collision normal
            Translation3d n = a.position.minus(b.position).div(dist);

            // Relative velocity
            Translation3d vRel = a.velocity.minus(b.velocity);

            // Velocity along normal
            double velAlongNormal = vRel.getX() * n.getX() +
                    vRel.getY() * n.getY() +
                    vRel.getZ() * n.getZ();

            // Do not resolve if velocities are separating
            if (velAlongNormal > 0)
                return;

            // Restitution (elastic = 1.0, balls are kinda bouncy but lose energy? Let's say
            // 0.9 for ball-ball)
            double e = 0.9;

            // Impulse scalar (masses are equal)
            double j = -(1 + e) * velAlongNormal;
            j /= 2.0; // 1/m + 1/m = 2 (assuming m=1)

            // Apply impulse
            Translation3d impulse = n.times(j);
            a.velocity = a.velocity.plus(impulse);
            b.velocity = b.velocity.minus(impulse);

            // Positional Correction (prevent sinking)
            double percent = 0.2; // Penetration percentage to correct
            double slop = 0.001; // Threshold
            double penetration = (radius * 2) - dist;
            if (penetration > slop) {
                Translation3d correction = n.times(penetration / 2.0 * percent);
                a.position = a.position.plus(correction);
                b.position = b.position.minus(correction);
            }
        }
    }
}
