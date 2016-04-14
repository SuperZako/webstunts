


/// <reference path="../cof/JConfig.ts"/>

/// <reference path="../data/SpanData.ts"/>

/// <reference path="../geom/Vector3D.ts"/>

/// <reference path="../math/JMath3D.ts"/>
/// <reference path="../math/JNumber3D.ts"/>

/// <reference path="../physics/MaterialProperties.ts"/>
/// <reference path="../physics/RigidBody.ts"/>




module jiglib {

    export class JAABox {
        minPos = null; // Vector3D
        maxPos = null; // Vector3D

        constructor() {
            this.clear();

        }

        get_sideLengths() {

            var pos = this.maxPos.clone();
            pos = pos.subtract(this.minPos);
            return pos;

        }

        get_centrePos() {

            var pos = this.minPos.clone();
            return JNumber3D.getScaleVector(pos.add(this.maxPos), 0.5);

        }

        getAllPoints() {

            var center, halfSide;
            var points;
            center = this.get_centrePos();
            halfSide = JNumber3D.getScaleVector(this.get_sideLengths(), 0.5);
            points = [];
            points[0] = center.add(new Vector3D(halfSide.x, -halfSide.y, halfSide.z));
            points[1] = center.add(new Vector3D(halfSide.x, halfSide.y, halfSide.z));
            points[2] = center.add(new Vector3D(-halfSide.x, -halfSide.y, halfSide.z));
            points[3] = center.add(new Vector3D(-halfSide.x, halfSide.y, halfSide.z));
            points[4] = center.add(new Vector3D(-halfSide.x, -halfSide.y, -halfSide.z));
            points[5] = center.add(new Vector3D(-halfSide.x, halfSide.y, -halfSide.z));
            points[6] = center.add(new Vector3D(halfSide.x, -halfSide.y, -halfSide.z));
            points[7] = center.add(new Vector3D(halfSide.x, halfSide.y, -halfSide.z));

            return points;

        }

        static get_edges() {
            return [
                new EdgeData(0, 1), new EdgeData(0, 2), new EdgeData(0, 6),
                new EdgeData(2, 3), new EdgeData(2, 4), new EdgeData(6, 7),
                new EdgeData(6, 4), new EdgeData(1, 3), new EdgeData(1, 7),
                new EdgeData(3, 5), new EdgeData(7, 5), new EdgeData(4, 5)];

        }

        getRadiusAboutCentre() {

            return 0.5 * (this.maxPos.subtract(this.minPos).get_length());

        }

        move(delta) {

            this.minPos.add(delta);
            this.maxPos.add(delta);

        }

        clear() {

            var huge = JMath3D.NUM_HUGE;
            this.minPos = new Vector3D(huge, huge, huge);
            this.maxPos = new Vector3D(-huge, -huge, -huge);

        }

        clone() {

            var aabb = new JAABox();
            aabb.minPos = this.minPos.clone();
            aabb.maxPos = this.maxPos.clone();
            return aabb;

        }

        addPoint(pos) {

            var tiny = JMath3D.NUM_TINY;
            if (pos.x < this.minPos.x) this.minPos.x = pos.x - tiny;
            if (pos.x > this.maxPos.x) this.maxPos.x = pos.x + tiny;
            if (pos.y < this.minPos.y) this.minPos.y = pos.y - tiny;
            if (pos.y > this.maxPos.y) this.maxPos.y = pos.y + tiny;
            if (pos.z < this.minPos.z) this.minPos.z = pos.z - tiny;
            if (pos.z > this.maxPos.z) this.maxPos.z = pos.z + tiny;

        }

        addBox(box) {

            var pts = box.getCornerPoints(box.get_currentState());
            this.addPoint(pts[0]);
            this.addPoint(pts[1]);
            this.addPoint(pts[2]);
            this.addPoint(pts[3]);
            this.addPoint(pts[4]);
            this.addPoint(pts[5]);
            this.addPoint(pts[6]);
            this.addPoint(pts[7]);

        }

        addSphere(sphere) {

            //if (sphere.get_currentState().position.x - sphere.get_radius() < _minPos.x) {
            this.minPos.x = (sphere.get_currentState().position.x - sphere.get_radius()) - 1;
            //}
            //if (sphere.get_currentState().position.x + sphere.get_radius() > _maxPos.x) {
            this.maxPos.x = (sphere.get_currentState().position.x + sphere.get_radius()) + 1;
            //}

            //if (sphere.get_currentState().position.y - sphere.get_radius() < _minPos.y) {
            this.minPos.y = (sphere.get_currentState().position.y - sphere.get_radius()) - 1;
            //}
            //if (sphere.get_currentState().position.y + sphere.get_radius() > _maxPos.y) {
            this.maxPos.y = (sphere.get_currentState().position.y + sphere.get_radius()) + 1;
            //}

            //if (sphere.get_currentState().position.z - sphere.get_radius() < _minPos.z) {
            this.minPos.z = (sphere.get_currentState().position.z - sphere.get_radius()) - 1;
            //}
            //if (sphere.get_currentState().position.z + sphere.get_radius() > _maxPos.z) {
            this.maxPos.z = (sphere.get_currentState().position.z + sphere.get_radius()) + 1;
            //}
            //trace("jaabox - add sphere:", _minPos.x,_minPos.y,_minPos.z,_maxPos.x,_maxPos.y,_maxPos.z);

            // todo: remove this code
            /*
        if (_minPos.x > _maxPos.x) {
            trace("minpos x ouch");
        }
        if (_minPos.y > _maxPos.y) {
            trace("minpos y ouch");
        }
        if (_minPos.z > _maxPos.z) {
            trace("minpos z ouch");
        }
        */


        }

        addCapsule(capsule) {

            var pos = capsule.getBottomPos(capsule.get_currentState());
            if (pos.x - capsule.get_radius() < this.minPos.x) {
                this.minPos.x = (pos.x - capsule.get_radius()) - 1;
            }
            if (pos.x + capsule.get_radius() > this.maxPos.x) {
                this.maxPos.x = (pos.x + capsule.get_radius()) + 1;
            }

            if (pos.y - capsule.get_radius() < this.minPos.y) {
                this.minPos.y = (pos.y - capsule.get_radius()) - 1;
            }
            if (pos.y + capsule.get_radius() > this.maxPos.y) {
                this.maxPos.y = (pos.y + capsule.get_radius()) + 1;
            }

            if (pos.z - capsule.get_radius() < this.minPos.z) {
                this.minPos.z = (pos.z - capsule.get_radius()) - 1;
            }
            if (pos.z + capsule.get_radius() > this.maxPos.z) {
                this.maxPos.z = (pos.z + capsule.get_radius()) + 1;
            }

            pos = capsule.getEndPos(capsule.get_currentState());
            if (pos.x - capsule.get_radius() < this.minPos.x) {
                this.minPos.x = (pos.x - capsule.get_radius()) - 1;
            }
            if (pos.x + capsule.get_radius() > this.maxPos.x) {
                this.maxPos.x = (pos.x + capsule.get_radius()) + 1;
            }

            if (pos.y - capsule.get_radius() < this.minPos.y) {
                this.minPos.y = (pos.y - capsule.get_radius()) - 1;
            }
            if (pos.y + capsule.get_radius() > this.maxPos.y) {
                this.maxPos.y = (pos.y + capsule.get_radius()) + 1;
            }

            if (pos.z - capsule.get_radius() < this.minPos.z) {
                this.minPos.z = (pos.z - capsule.get_radius()) - 1;
            }
            if (pos.z + capsule.get_radius() > this.maxPos.z) {
                this.maxPos.z = (pos.z + capsule.get_radius()) + 1;
            }

        }

        addSegment(seg) {

            this.addPoint(seg.origin);
            this.addPoint(seg.getEnd());

        }

        overlapTest(box) {

            return (!((this.minPos.z >= box.maxPos.z) ||
            (this.maxPos.z <= box.minPos.z) ||
            (this.minPos.y >= box.maxPos.y) ||
            (this.maxPos.y <= box.minPos.y) ||
            (this.minPos.x >= box.maxPos.x) ||
            (this.maxPos.x <= box.minPos.x)));

        }

        isPointInside(pos) {

            return ((pos.x >= this.minPos.x) &&
                (pos.x <= this.maxPos.x) &&
                (pos.y >= this.minPos.y) &&
                (pos.y <= this.maxPos.y) &&
                (pos.z >= this.minPos.z) &&
                (pos.z <= this.maxPos.z));

        }

        segmentAABoxOverlap(seg) {

            var jDir, kDir, i, iFace;
            var frac, dist0, dist1, tiny = JMath3D.NUM_TINY;

            var pt, minPosArr, maxPosArr, p0, p1, faceOffsets;
            minPosArr = JNumber3D.toArray(this.minPos);
            maxPosArr = JNumber3D.toArray(this.maxPos);
            p0 = JNumber3D.toArray(seg.origin);
            p1 = JNumber3D.toArray(seg.getEnd());
            for (i = 0; i < 3; i++) {
                jDir = (i + 1) % 3;
                kDir = (i + 2) % 3;
                faceOffsets = [[minPosArr[i], maxPosArr[i]]];

                for (iFace = 0; iFace < 2; iFace++) {
                    dist0 = p0[i] - faceOffsets[iFace];
                    dist1 = p1[i] - faceOffsets[iFace];
                    frac = -1;
                    if (dist0 * dist1 < -tiny)
                        frac = -dist0 / (dist1 - dist0);
                    else if (Math.abs(dist0) < tiny)
                        frac = 0;
                    else if (Math.abs(dist1) < tiny)
                        frac = 1;

                    if (frac >= 0) {
                        pt = JNumber3D.toArray(seg.getPoint(frac));
                        if ((pt[jDir] > minPosArr[jDir] - tiny) &&
                            (pt[jDir] < maxPosArr[jDir] + tiny) &&
                            (pt[kDir] > minPosArr[kDir] - tiny) &&
                            (pt[kDir] < maxPosArr[kDir] + tiny)) {
                            return true;
                        }
                    }
                }
            }
            return false;

        }
    }
}