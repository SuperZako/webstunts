

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

    export class CollDetectSpherePlane extends CollDetectFunctor {

        constructor() {
            super("SpherePlane", "SPHERE", "PLANE");
            //this.name = "SpherePlane";
            //this.type0 = "SPHERE";
            //this.type1 = "PLANE";

        }



        collDetect(info, collArr) {

            var tempBody;
            if (info.body0.get_type() == "PLANE") {
                tempBody = info.body0;
                info.body0 = info.body1;
                info.body1 = tempBody;
            }

            var sphere = info.body0;
            var plane = info.body1;

            var oldDist, newDist, depth;
            oldDist = plane.pointPlaneDistance(sphere.get_oldState().position);
            newDist = plane.pointPlaneDistance(sphere.get_currentState().position);

            if (Math.min(newDist, oldDist) > sphere.get_boundingSphere() + JConfig.collToll) {
                info.body0.removeCollideBodies(info.body1);
                info.body1.removeCollideBodies(info.body0);
                return;
            }

            var collPts = [];
            var cpInfo;
            depth = sphere.get_radius() - oldDist;

            var worldPos = sphere.get_oldState().position.subtract(JNumber3D.getScaleVector(plane.get_normal(), sphere.get_radius()));
            cpInfo = new CollPointInfo();
            cpInfo.r0 = worldPos.subtract(sphere.get_oldState().position);
            cpInfo.r1 = worldPos.subtract(plane.get_oldState().position);
            cpInfo.initialPenetration = depth;
            collPts[0] = cpInfo;

            var collInfo = new CollisionInfo();
            collInfo.objInfo = info;
            collInfo.dirToBody = plane.get_normal().clone();
            collInfo.pointInfo = collPts;

            var mat = new MaterialProperties();
            mat.restitution = 0.5 * (sphere.get_material().restitution + plane.get_material().restitution);
            mat.friction = 0.5 * (sphere.get_material().friction + plane.get_material().friction);
            collInfo.mat = mat;
            collArr.push(collInfo);
            info.body0.collisions.push(collInfo);
            info.body1.collisions.push(collInfo);
            info.body0.addCollideBody(info.body1);
            info.body1.addCollideBody(info.body0);

        }


    }
}