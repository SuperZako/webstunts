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

namespace jiglib {

    export class CollDetectBoxPlane extends CollDetectFunctor {

        constructor() {
            super("BoxPlane", "BOX", "PLANE");

        }

        collDetect(info, collArr) {

            var tempBody;
            if (info.body0.get_type() == "PLANE") {
                tempBody = info.body0;
                info.body0 = info.body1;
                info.body1 = tempBody;
            }

            var box = info.body0;
            var plane = info.body1;

            var centreDist = plane.pointPlaneDistance(box.get_currentState().position);
            if (centreDist > box.get_boundingSphere() + JConfig.collToll)
                return;

            var newPts = box.getCornerPoints(box.get_currentState());
            var oldPts = box.getCornerPoints(box.get_oldState());
            var collPts = [];
            var cpInfo;
            var newPt, oldPt;
            var newDepth, oldDepth;
            var newPts_length = newPts.length;

            for (var i = 0; i < newPts_length; i++) {
                newPt = newPts[i];
                oldPt = oldPts[i];
                newDepth = -1 * plane.pointPlaneDistance(newPt);
                oldDepth = -1 * plane.pointPlaneDistance(oldPt);

                if (Math.max(newDepth, oldDepth) > -JConfig.collToll) {
                    cpInfo = new CollPointInfo();
                    cpInfo.r0 = oldPt.subtract(box.get_oldState().position);
                    cpInfo.r1 = oldPt.subtract(plane.get_oldState().position);
                    cpInfo.initialPenetration = oldDepth;
                    collPts.push(cpInfo);
                }
            }

            if (collPts.length > 0) {
                var collInfo = new CollisionInfo();
                collInfo.objInfo = info;
                collInfo.dirToBody = plane.get_normal().clone();
                collInfo.pointInfo = collPts;

                var mat = new MaterialProperties();
                mat.restitution = 0.5 * (box.get_material().restitution + plane.get_material().restitution);
                mat.friction = 0.5 * (box.get_material().friction + plane.get_material().friction);
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