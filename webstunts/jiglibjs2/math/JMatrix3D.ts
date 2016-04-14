
/// <reference path="../geom/Vector3D.ts"/>
/// <reference path="../geom/Matrix3D.ts"/>

module jiglib {

    export class JMatrix3D {

        static getTranslationMatrix(x, y, z) {

            var matrix3D = new Matrix3D();
            matrix3D.appendTranslation(x, y, z);
            return matrix3D;

        }

        static getScaleMatrix(x, y, z) {

            var matrix3D = new Matrix3D();
            matrix3D.prependScale(x, y, z);
            return matrix3D;

        }

        static getRotationMatrix(x, y, z, degree, pivotPoint= null) {

            var matrix3D = new Matrix3D();
            matrix3D.appendRotation(degree, new Vector3D(x, y, z), pivotPoint);
            return matrix3D;

        }

        static getInverseMatrix(m) {

            var matrix3D = m.clone();
            matrix3D.invert();
            return matrix3D;

        }

        static getTransposeMatrix(m) {

            var matrix3D = m.clone();
            matrix3D.transpose();
            return matrix3D;

        }

        static getAppendMatrix3D(a, b) {

            var matrix3D = a.clone();
            matrix3D.append(b);
            return matrix3D;

        }

        static getSubMatrix(a, b) {

            var ar = a.get_rawData();
            var br = b.get_rawData();
            return new Matrix3D([[
                ar[0] - br[0],
                ar[1] - br[1],
                ar[2] - br[2],
                ar[3] - br[3],
                ar[4] - br[4],
                ar[5] - br[5],
                ar[6] - br[6],
                ar[7] - br[7],
                ar[8] - br[8],
                ar[9] - br[9],
                ar[10] - br[10],
                ar[11] - br[11],
                ar[12] - br[12],
                ar[13] - br[13],
                ar[14] - br[14],
                ar[15] - br[15]
            ]]);

        }

        static getRotationMatrixAxis(degree, rotateAxis) {

            var matrix3D = new Matrix3D();
            matrix3D.appendRotation(degree, rotateAxis ? rotateAxis : Vector3D.X_AXIS);
            return matrix3D;

        }

        static getCols(matrix3D) {

            var rawData = matrix3D.get_rawData();
            var cols = [];

            cols[0] = new Vector3D(rawData[0], rawData[4], rawData[8]);
            cols[1] = new Vector3D(rawData[1], rawData[5], rawData[9]);
            cols[2] = new Vector3D(rawData[2], rawData[6], rawData[10]);

            return cols;

        }

    }
}