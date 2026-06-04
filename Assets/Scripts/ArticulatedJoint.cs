using UnityEngine;
using UnityEngine.Serialization;


namespace ArticulatedBody
{
    public enum JointType
    {
        Revolute,   // Rotates around axis
        Prismatic,  // Slides along axis
        Spherical,  // Ball socket
    }

    [System.Serializable]
    public class ArticulatedJoint
    {
        public JointType type = JointType.Revolute;

        /**  Joint axis in parent body local frame */
        public Vector3 axis = Vector3.right;

        /** Index of parent body. -1 = world root */
        public int parentIndex = -1;

        /** Length of the rigid link */
        public float linkLength = 1.0f;

        /** Constraints / Limits */
        public bool hasLimits = false;

        public float qMin = -Mathf.PI;
        public float qMax = Mathf.PI;

        /** Viscous damping coefficient */
        public float damping = 0.0f;

        /** Generalised coordinates: angle or displacement. */
        public float q = 0.0f;

        /** Generalised velocity: (rad/s or m/s). */
        public float qDot = 0.0f;

        /** Generalized force: torque (Nm) or force (N) */
        public float tau = 0.0f;

        public Vector3 LocalPosition { get; private set; }
        public Quaternion LocalRotation { get; private set; }

        /** Computes the local transform contributed by this joint. */
        public (Vector3 localPosition, Quaternion localRotation) GetTransform()
        {
            Vector3 pos;
            Quaternion rot;

            switch (type)
            {
                case JointType.Revolute:
                case JointType.Spherical:
                    rot = Quaternion.AngleAxis(q * Mathf.Rad2Deg, axis.normalized);
                    pos = rot * (axis.normalized * linkLength);
                    return (pos, rot);

                case JointType.Prismatic:
                    rot = Quaternion.identity;
                    pos = axis.normalized * (linkLength + q);
                    return (pos, rot);

                default:
                    return (Vector3.zero, Quaternion.identity);
            }
        }

        /** Semi-implicit Euler integration. */
        public void Integrate(float dt, float qDotDot = 0.0f)
        {
            float acceleration = qDotDot - damping * qDot;

            qDot += acceleration * dt;

            if (hasLimits)
                qDot = ClampedVelocity(dt, qDot);

            q += qDot * dt;

            if (hasLimits)
                q = Mathf.Clamp(q, qMin, qMax);

            (LocalPosition, LocalRotation) = GetTransform();
        }

        /** Resets joint, clearing all dynamics state. */
        public void Reset()
        {
            q = 0.0f;
            qDot = 0.0f;
            tau = 0.0f;

            (LocalPosition, LocalRotation) = GetTransform();
        }

        /** Clamps velocity. */
        private float ClampedVelocity(float dt, float velocity)
        {
            float nextQ = q + velocity * dt;

            if (nextQ <= qMin && velocity < 0.0f) return 0.0f;
            if (nextQ >= qMax && velocity > 0.0f) return 0.0f;

            return velocity;
        }
    }
}
