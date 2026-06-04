using UnityEngine;

namespace ArticulatedBody
{
    [System.Serializable]
    public class ArticulatedRigidBody
    {
        /** Total mass in kg. */
        public float mass = 1.0f;

        /** Inertia tensor expressed in the body's local frame. */
        public Matrix4x4 inertiaTensor = Matrix4x4.identity;

        /** Centre of mass offset relative to the joint origin. */
        public Vector3 comOffset = Vector3.zero;

        /** Position of the joint origin in world space. */
        public Vector3 position = Vector3.zero;

        /** Orientation in world space. */
        public Quaternion rotation = Quaternion.identity;
        
        /** Linear velocity in world space in m/s */
        public Vector3 velocity = Vector3.zero;
        
        /** Angular velocity in world space in rad/s */
        public Vector3 angularVelocity = Vector3.zero;

        /** Centre-of-mass position in world space. */
        public Vector3 WorldCoM => position + rotation * comOffset;

        /** Returns the inertia tensor rotated into world space. */
        public Matrix4x4 WorldInertiaTensor()
        {
            Matrix4x4 R = Matrix4x4.Rotate(rotation);
            Matrix4x4 RT = R.transpose;
            return R * inertiaTensor * RT;
        }

        /** Translational + rotational kinetic energy */
        public float KineticEnergy()
        {
            float translational = 0.5f * mass * velocity.sqrMagnitude;

            Matrix4x4 I = WorldInertiaTensor();
            Vector3 Iw = new Vector3(
                I.GetRow(0).x * angularVelocity.x +
                I.GetRow(0).y * angularVelocity.y +
                I.GetRow(0).z * angularVelocity.z,
                I.GetRow(1).x * angularVelocity.x +
                I.GetRow(1).y * angularVelocity.y +
                I.GetRow(1).z * angularVelocity.z,
                I.GetRow(2).x * angularVelocity.x +
                I.GetRow(2).y * angularVelocity.y +
                I.GetRow(2).z * angularVelocity.z
            );

            float rotational = 0.5f * Vector3.Dot(angularVelocity, Iw);

            return translational + rotational;
        }

        /** Returns the net torque produced by applying `force` at `worldPoint` */
        public Vector3 TorqueFromForce(Vector3 force, Vector3 worldPoint)
        {
            Vector3 r = worldPoint - WorldCoM;
            return Vector3.Cross(r, force);
        }

        /** Sets world-space pose directly. */
        public void SetPose(Vector3 worldPosition, Quaternion worldRotation)
        {
            position = worldPosition;
            rotation = worldRotation;
        }
        
        /** Resets to identity pose. */
        public void Reset()
        {
            position = Vector3.zero;
            rotation = Quaternion.identity;
            velocity = Vector3.zero;
            angularVelocity = Vector3.zero;
        }

        /** Inertia tensor for a uniform solid box (w × h × d). */
        public static Matrix4x4 InertiaTensorBox(float mass, float w, float h, float d)
        {
            float c = mass / 12.0f;
            Matrix4x4 I = Matrix4x4.zero;
            I[0, 0] = c * (h * h + d * d);
            I[1, 1] = c * (w * w + d * d);
            I[2, 2] = c * (w * w + h * h);
            I[3, 3] = 1.0f;
            return I;
        }
        
        /** Inertia tensor for a uniform solid cylinder (radius r, height h), axis = Y. */
        public static Matrix4x4 InertiaTensorCylinder(float mass, float r, float h)
        {
            float Ixy = (mass / 12.0f) * (3 * r * r + h * h);
            float Iz  = 0.5f * mass * r * r;
            Matrix4x4 I = Matrix4x4.zero;
            I[0, 0] = Ixy;
            I[1, 1] = Iz;
            I[2, 2] = Ixy;
            I[3, 3] = 1.0f;
            return I;
        }

        /** Inertia tensor for a uniform solid sphere. */
        public static Matrix4x4 InertiaTensorSphere(float mass, float r)
        {
            float v = (2.0f / 5.0f) * mass * r * r;
            Matrix4x4 I = Matrix4x4.zero;
            I[0, 0] = v;
            I[1, 1] = v;
            I[2, 2] = v;
            I[3, 3] = 1.0f;
            return I;
        }
    }
}
