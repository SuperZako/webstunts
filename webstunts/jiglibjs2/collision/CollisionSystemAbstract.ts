



/// <reference path="CollDetectBoxBox.ts"/>
/// <reference path="CollDetectSphereBox.ts"/>
/// <reference path="CollDetectCapsuleBox.ts"/>
/// <reference path="CollDetectBoxPlane.ts"/>

/// <reference path="CollDetectBoxTerrain.ts"/>
/// <reference path="CollDetectBoxMesh.ts"/>
/// <reference path="CollDetectSphereSphere.ts"/>
/// <reference path="CollDetectSphereCapsule.ts"/>

/// <reference path="CollDetectSpherePlane.ts"/>
/// <reference path="CollDetectSphereTerrain.ts"/>
/// <reference path="CollDetectSphereMesh.ts"/>
/// <reference path="CollDetectCapsuleCapsule.ts"/>

/// <reference path="CollDetectCapsulePlane.ts"/>
/// <reference path="CollDetectCapsuleTerrain.ts"/>

module jiglib {

    export class CollisionSystemAbstract {
        detectionFunctors: { [index: string]: CollDetectFunctor } = {}; // Dictionary
        collBody: RigidBody[] = []; // RigidBody
        _numCollisionsChecks = 0; // uint
        startPoint: Vector3D = null; // Vector3D

        constructor() {
            //this.detectionFunctors = [];
            this.detectionFunctors["BOX_BOX"] = new CollDetectBoxBox();
            this.detectionFunctors["BOX_SPHERE"] = new CollDetectSphereBox();
            this.detectionFunctors["BOX_CAPSULE"] = new CollDetectCapsuleBox();
            this.detectionFunctors["BOX_PLANE"] = new CollDetectBoxPlane();
            this.detectionFunctors["BOX_TERRAIN"] = new CollDetectBoxTerrain();
            this.detectionFunctors["BOX_TRIANGLEMESH"] = new CollDetectBoxMesh();
            this.detectionFunctors["SPHERE_BOX"] = new CollDetectSphereBox();
            this.detectionFunctors["SPHERE_SPHERE"] = new CollDetectSphereSphere();
            this.detectionFunctors["SPHERE_CAPSULE"] = new CollDetectSphereCapsule();
            this.detectionFunctors["SPHERE_PLANE"] = new CollDetectSpherePlane();
            this.detectionFunctors["SPHERE_TERRAIN"] = new CollDetectSphereTerrain();
            this.detectionFunctors["SPHERE_TRIANGLEMESH"] = new CollDetectSphereMesh();
            this.detectionFunctors["CAPSULE_CAPSULE"] = new CollDetectCapsuleCapsule();
            this.detectionFunctors["CAPSULE_BOX"] = new CollDetectCapsuleBox();
            this.detectionFunctors["CAPSULE_SPHERE"] = new CollDetectSphereCapsule();
            this.detectionFunctors["CAPSULE_PLANE"] = new CollDetectCapsulePlane();
            this.detectionFunctors["CAPSULE_TERRAIN"] = new CollDetectCapsuleTerrain();
            this.detectionFunctors["PLANE_BOX"] = new CollDetectBoxPlane();
            this.detectionFunctors["PLANE_SPHERE"] = new CollDetectSpherePlane();
            this.detectionFunctors["PLANE_CAPSULE"] = new CollDetectCapsulePlane();
            this.detectionFunctors["TERRAIN_SPHERE"] = new CollDetectSphereTerrain();
            this.detectionFunctors["TERRAIN_BOX"] = new CollDetectBoxTerrain();
            this.detectionFunctors["TERRAIN_CAPSULE"] = new CollDetectCapsuleTerrain();
            this.detectionFunctors["TRIANGLEMESH_SPHERE"] = new CollDetectSphereMesh();
            this.detectionFunctors["TRIANGLEMESH_BOX"] = new CollDetectBoxMesh();

        }

        addCollisionBody(body) {

            if (this.collBody.indexOf(body) < 0)
                this.collBody.push(body);

        }

        removeCollisionBody(body) {

            if (this.collBody.indexOf(body) >= 0)
                this.collBody.splice(this.collBody.indexOf(body), 1);

        }

        removeAllCollisionBodies() {

            this.collBody.length = 0;

        }

        detectCollisions(body: RigidBody, collArr) {

            if (!body.isActive)
                return;

            for (let _collBody of this.collBody) {
                if (body == _collBody) {
                    continue;
                }
                if (this.checkCollidables(body, _collBody) && this.detectionFunctors[body.get_type() + "_" + _collBody.get_type()] != undefined) {
                    let info = new CollDetectInfo();
                    info.body0 = body;
                    info.body1 = _collBody;
                    let fu = this.detectionFunctors[info.body0.get_type() + "_" + info.body1.get_type()];
                    fu.collDetect(info, collArr);
                }
            }

        }

        detectAllCollisions(bodies, collArr) {


        }

        collisionSkinMoved(colBody) {

            // used for grid

        }

        segmentIntersect(out: CollOutBodyData, seg: JSegment, ownerBody: RigidBody) {

            out.frac = JMath3D.NUM_HUGE;
            out.position = new Vector3D();
            out.normal = new Vector3D();

            var obj = new CollOutBodyData();
            for (let _collBody of this.collBody) {
                if (_collBody != ownerBody && this.segmentBounding(seg, _collBody)) {
                    if (_collBody.segmentIntersect(obj, seg, _collBody.get_currentState())) {
                        if (obj.frac < out.frac) {
                            out.position = obj.position;
                            out.normal = obj.normal;
                            out.frac = obj.frac;
                            out.rigidBody = _collBody;
                        }
                    }
                }
            }

            if (out.frac > 1)
                return false;

            if (out.frac < 0) {
                out.frac = 0;
            }
            else if (out.frac > 1) {
                out.frac = 1;
            }

            return true;

        }

        segmentBounding(seg, obj) {

            var pos = seg.getPoint(0.5);
            var r = seg.delta.get_length() / 2;

            var num1 = pos.subtract(obj.get_currentState().position).get_length();
            var num2 = r + obj.get_boundingSphere();

            if (num1 <= num2)
                return true;
            else
                return false;

        }

        get_numCollisionsChecks() {

            return this._numCollisionsChecks;

        }

        checkCollidables(body0: RigidBody, body1: RigidBody) {

            if (body0.get_nonCollidables().length == 0 && body1.get_nonCollidables().length == 0)
                return true;

            if (body0.get_nonCollidables().indexOf(body1) > -1)
                return false;

            if (body1.get_nonCollidables().indexOf(body0) > -1)
                return false;

            return true;
        }
    }
}