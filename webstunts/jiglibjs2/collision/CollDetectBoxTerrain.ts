﻿
/// <reference path="../cof/JConfig.ts"/>

/// <reference path="../data/SpanData.ts"/>
/// <reference path="../data/CollOutBodyData.ts"/>

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

    export class CollDetectBoxTerrain extends CollDetectFunctor {
        constructor() {
            super("BoxTerrain", "BOX", "TERRAIN");
        }

        collDetect(info, collArr) {

            var tempBody;
            if (info.body0.get_type() == "TERRAIN") {
                tempBody = info.body0;
                info.body0 = info.body1;
                info.body1 = tempBody;
            }

            var box = info.body0;
            var terrain = info.body1;

            var oldPts = box.getCornerPoints(box.get_oldState());
            var newPts = box.getCornerPoints(box.get_currentState());
            var collNormal = new Vector3D();

            var obj;
            var dist;
            var newPt;
            var oldPt;

            var collPts = [];
            var cpInfo;

            for (var i = 0; i < 8; i++) {
                newPt = newPts[i];
                obj = terrain.getHeightAndNormalByPoint(newPt);

                if (obj.height < JConfig.collToll) {
                    oldPt = oldPts[i];
                    dist = terrain.getHeightByPoint(oldPt);
                    collNormal = collNormal.add(obj.normal);
                    cpInfo = new CollPointInfo();
                    cpInfo.r0 = oldPt.subtract(box.get_oldState().position);
                    cpInfo.r1 = oldPt.subtract(terrain.get_oldState().position);
                    cpInfo.initialPenetration = -dist;
                    collPts.push(cpInfo);
                }
            }

            if (collPts.length > 0) {
                collNormal.normalize();

                var collInfo = new CollisionInfo();
                collInfo.objInfo = info;
                collInfo.dirToBody = collNormal;
                collInfo.pointInfo = collPts;

                var mat = new MaterialProperties();
                mat.restitution = 0.5 * (box.get_material().restitution + terrain.get_material().restitution);
                mat.friction = 0.5 * (box.get_material().friction + terrain.get_material().friction);
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