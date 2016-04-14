
/// <reference path="../geom/Vector3D.ts"/>

module jiglib {

    export class JNumber3D {




        static toArray(v) {

            var arr = [];
            arr[0] = v.x;
            arr[1] = v.y;
            arr[2] = v.z;
            return arr;

        }

        static copyFromArray(v, arr) {

            if (arr.length >= 3) {
                v.x = arr[0];
                v.y = arr[1];
                v.z = arr[2];
            }

        }

        static getScaleVector(v, s) {

            return new Vector3D(v.x * s, v.y * s, v.z * s, v.w);

        }

        static getDivideVector(v, w) {

            if (w != 0) {
                return new Vector3D(v.x / w, v.y / w, v.z / w);
            }
            else {
                return new Vector3D(0, 0, 0);
            }

        }

        static getNormal(v0, v1, v2) {

            var E = v1.clone();
            var F = v2.clone();
            var N = E.subtract(v0).crossProduct(F.subtract(v1));
            N.normalize();

            return N;

        }
    }
}