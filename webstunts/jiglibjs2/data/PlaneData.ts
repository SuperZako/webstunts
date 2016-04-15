
/// <reference path="../cof/JConfig.ts"/>

/// <reference path="../data/SpanData.ts"/>

/// <reference path="../geom/Vector3D.ts"/>

/// <reference path="../math/JMath3D.ts"/>
/// <reference path="../math/JNumber3D.ts"/>

/// <reference path="../physics/MaterialProperties.ts"/>
/// <reference path="../physics/RigidBody.ts"/>



module jiglib {

    export class PlaneData {
        private _position = new Vector3D();
        private _normal = new Vector3D(0, 1, 0);
        private _distance = 0;
        constructor() { }

        get_position() {

            return this._position;

        }

        get_normal() {

            return this._normal;

        }

        get_distance() {

            return this._distance;

        }

        pointPlaneDistance(pt) {

            return this._normal.dotProduct(pt) - this._distance;

        }

        setWithNormal(pos, nor) {

            this._position = pos.clone();
            this._normal = nor.clone();
            this._distance = pos.dotProduct(nor);

        }

        setWithPoint(pos0: Vector3D, pos1: Vector3D, pos2: Vector3D) {

            this._position = pos0.clone();

            var dr1 = pos1.subtract(pos0);
            var dr2 = pos2.subtract(pos0);
            this._normal = dr1.crossProduct(dr2);

            var nLen = this._normal.get_length();
            if (nLen < JMath3D.NUM_TINY) {
                this._normal = new Vector3D(0, 1, 0);
                this._distance = 0;
            } else {
                this._normal.scaleBy(1 / nLen);
                this._distance = pos0.dotProduct(this._normal);
            }

        }

    }
}