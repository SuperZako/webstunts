﻿

/// <reference path="../cof/JConfig.ts"/>

/// <reference path="../data/SpanData.ts"/>

/// <reference path="../geom/Vector3D.ts"/>

/// <reference path="../math/JMath3D.ts"/>
/// <reference path="../math/JNumber3D.ts"/>

/// <reference path="../physics/MaterialProperties.ts"/>
/// <reference path="../physics/RigidBody.ts"/>

/// <reference path="CollDetectFunctor.ts"/>
/// <reference path="CollPointInfo.ts"/>
/// <reference path="CollisionInfo.ts"/>
/// <reference path="CollDetectInfo.ts"/>


module jiglib {
    export class CollDetectCapsulePlane extends CollDetectFunctor {
        constructor() {
            super("CapsulePlane", "CAPSULE", "PLANE");
        }

        collDetect(info, collArr) {

       
            if (info.body0.get_type() == "PLANE") {
                let tempBody = info.body0;
                info.body0 = info.body1;
                info.body1 = tempBody;
            }

            var capsule = info.body0;
            var plane = info.body1;

            var collPts = [];
            var cpInfo;

            var oldPos = capsule.getBottomPos(capsule.get_oldState());
            var oldDist = plane.pointPlaneDistance(oldPos);
            var newPos = capsule.getBottomPos(capsule.get_currentState());
            var newDist = plane.pointPlaneDistance(newPos);

            if (Math.min(oldDist, newDist) < capsule.get_radius() + JConfig.collToll) {
                var oldDepth = capsule.get_radius() - oldDist;
                var worldPos = oldPos.subtract(JNumber3D.getScaleVector(plane.get_normal(), capsule.get_radius()));

                cpInfo = new CollPointInfo();
                cpInfo.r0 = worldPos.subtract(capsule.get_oldState().position);
                cpInfo.r1 = worldPos.subtract(plane.get_oldState().position);
                cpInfo.initialPenetration = oldDepth;
                collPts.push(cpInfo);
            }

            oldPos = capsule.getEndPos(capsule.get_oldState());
            newPos = capsule.getEndPos(capsule.get_currentState());
            oldDist = plane.pointPlaneDistance(oldPos);
            newDist = plane.pointPlaneDistance(newPos);
            if (Math.min(oldDist, newDist) < capsule.get_radius() + JConfig.collToll) {
                oldDepth = capsule.get_radius() - oldDist;
                worldPos = oldPos.subtract(JNumber3D.getScaleVector(plane.get_normal(), capsule.get_radius()));

                cpInfo = new CollPointInfo();
                cpInfo.r0 = worldPos.subtract(capsule.get_oldState().position);
                cpInfo.r1 = worldPos.subtract(plane.get_oldState().position);
                cpInfo.initialPenetration = oldDepth;
                collPts.push(cpInfo);
            }

            if (collPts.length > 0) {
                var collInfo = new CollisionInfo();
                collInfo.objInfo = info;
                collInfo.dirToBody = plane.get_normal().clone();
                collInfo.pointInfo = collPts;

                var mat = new MaterialProperties();
                mat.restitution = 0.5 * (capsule.get_material().restitution + plane.get_material().restitution);
                mat.friction = 0.5 * (capsule.get_material().friction + plane.get_material().friction);
                collInfo.mat = mat;
                collArr.push(collInfo);
                info.body0.collisions.push(collInfo);
                info.body1.collisions.push(collInfo);
                info.body0.addCollideBody(info.body1);
                info.body1.addCollideBody(info.body0);
            } else {
                info.body0.removeCollideBodies(info.body1);
                info.body1.removeCollideBodies(info.body0);
            }

        }
    }
}