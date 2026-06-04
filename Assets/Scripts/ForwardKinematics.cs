using UnityEngine;

namespace ArticulatedBody
{
    public static class ForwardKinematics
    {
        public static void BodyTransform(ArticulatedJoint[] joints, ArticulatedRigidBody[] bodies, int i)
        {
            ArticulatedJoint joint = joints[i];
            var (localPos, localRot) = joint.GetTransform();

            if (joint.parentIndex < 0)
            {
                bodies[i].position = localPos;
                bodies[i].rotation = localRot;
            }
            else
            {
                ArticulatedRigidBody parent    = bodies[joint.parentIndex];
                bodies[i].position  = parent.position + parent.rotation * localPos;
                bodies[i].rotation  = parent.rotation * localRot;
            }
        }
        
        public static void UpdateAll(ArticulatedJoint[] joints, ArticulatedRigidBody[] bodies)
        {
            for (int i = 0; i < joints.Length; i++)
                BodyTransform(joints, bodies, i);
        }
        
        public static Vector3 EndEffectorPos(ArticulatedJoint[] joints, ArticulatedRigidBody[] bodies)
        {
            UpdateAll(joints, bodies);
            return bodies[bodies.Length - 1].position;
        }
        
        public static float[] JacobianColumn(ArticulatedJoint[] joints, ArticulatedRigidBody[] bodies, int i)
        {
            UpdateAll(joints, bodies);
            return JacobianColumnInternal(joints, bodies, i);
        }
        
        public static float[,] Jacobian(ArticulatedJoint[] joints, ArticulatedRigidBody[] bodies)
        {
            int n        = joints.Length;
            float[,] J   = new float[6, n];

            UpdateAll(joints, bodies);

            for (int i = 0; i < n; i++)
            {
                float[] col = JacobianColumnInternal(joints, bodies, i);
                for (int row = 0; row < 6; row++)
                    J[row, i] = col[row];
            }

            return J;
        }

        private static float[] JacobianColumnInternal(ArticulatedJoint[] joints, ArticulatedRigidBody[] bodies, int i)
        {
            Vector3 endPos    = bodies[bodies.Length - 1].position;
            Vector3 jointPos  = bodies[i].position;
            Vector3 worldAxis = bodies[i].rotation * joints[i].axis.normalized;

            Vector3 v, w;

            switch (joints[i].type)
            {
                case JointType.Revolute:
                case JointType.Spherical:
                    v = Vector3.Cross(worldAxis, endPos - jointPos);
                    w = worldAxis;
                    break;

                case JointType.Prismatic:
                    v = worldAxis;
                    w = Vector3.zero;
                    break;

                default:
                    v = Vector3.zero;
                    w = Vector3.zero;
                    break;
            }

            return new float[] { v.x, v.y, v.z, w.x, w.y, w.z };
        }
    }
}