using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using ZO.Math;

namespace ZO.Editor {
    [CustomEditor(typeof(ZO.Physics.ZOPrismaticJoint))]

    public class ZOPrismaticJointEditor : UnityEditor.Editor {

        // private bool _isConfigurableJointHidden = false;
        // private ZO.Physics.ZOPrismaticJoint _prismaticJoint;
        // public override void OnInspectorGUI() {
        //     DrawDefaultInspector();

        //     _prismaticJoint = target as ZO.Physics.ZOPrismaticJoint;

        // }


        private void OnEnable() {
            ZO.Physics.ZOPrismaticJoint prismaticJoint = target as ZO.Physics.ZOPrismaticJoint;
            if (prismaticJoint.GetComponent<ConfigurableJoint>()) {

                prismaticJoint.GetComponent<ConfigurableJoint>().hideFlags = HideFlags.HideInInspector;
            }            
        }


    }

}

