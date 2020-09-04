using UnityEngine;
using ZO.Document;

namespace ZO.Physics {

    /// <summary>
    /// Defines a standard interface to access joints.
    /// </summary>
    public interface ZOJointInterface {


        /// <summary>
        /// Each joint is uniquely identified by its name.
        /// </summary>
        /// <value></value>
        string Name { get; }

        /// <summary>
        /// Type of joint.  For example "joint.hinge".
        /// </summary>
        /// <value></value>
        string Type { get; }


        /// <summary>
        /// The joint position.  
        /// 
        /// For a hinge/revolute it would be the angle in radians.
        /// 
        /// For a linear/prismatic it would be the distance from center in meters.
        /// </summary>
        /// <value></value>
        float Position { get; set; }

        /// <summary>
        /// The velocity of the joint (rad/s or m/s)
        /// </summary>
        /// <value></value>
        float Velocity { get; set; }


        /// <summary>
        /// The effort that is applied to the joint (Nm or N)
        /// </summary>
        /// <value></value>
        float Effort { get; set; }

        /// <summary>
        /// The connected rigid body.  If null then it is the world.
        /// </summary>
        /// <value></value>
        Rigidbody ConnectedBody { get; set; }

        /// <summary>
        /// The connected ZOSim Occurrence.  Being null does not necessarily mean anything.
        /// </summary>
        /// <value></value>
        ZOSimOccurrence ConnectedOccurrence { get;  }

    }
}