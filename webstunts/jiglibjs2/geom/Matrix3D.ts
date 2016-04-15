
/// <reference path="../../glMatrix.ts"/>

/// <reference path="../geom/Vector3D.ts"/>

module jiglib {

    export class Matrix3D {
        constructor(v = null) {
            if (v) {
                this._rawData = mat4.create(v);
            }
            else {
                this._rawData = mat4.create([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]);
            }
        }


        _rawData = null;


        // A Vector of 16 Numbers, where every four elements can be a row or a column of a 4x4 matrix.
        get_rawData() {
            return this._rawData;
        }

        // [read-only] A Number that determines whether a matrix is invertible.
        // return void
        get_determinant() {
            return mat4.determinant(this._rawData);
        }

        // Appends the matrix by multiplying another Matrix3D object by the current Matrix3D object.
        // return void
        append(m) {
            mat4.multiply(this._rawData, m._rawData);
        }

        // Appends an incremental rotation to a Matrix3D object.
        // return void
        appendRotation(angle: number, axis: Vector3D, pivot: Vector3D = null) {
            // angle = angle/(3.14159*2);	
            angle = angle * Math.PI / 180;
            if (pivot) {
                var npivot = pivot.clone().negate();
                this.appendTranslation(npivot.x, npivot.y, npivot.z);
            }
            var naxis = axis.clone().negate();
            mat4.rotate(this._rawData, angle, [naxis.x, naxis.y, naxis.z]);
            if (pivot) {
                this.appendTranslation(pivot.x, pivot.y, pivot.z);
            }
        }


        // Appends an incremental scale change along the x, y, and z axes to a Matrix3D object.
        // return void
        appendScale(x, y, z) {
            mat4.scale(this._rawData, [x, y, z]);
        }


        // Appends an incremental translation, a repositioning along the x, y, and z axes, to a Matrix3D object.
        // return void
        appendTranslation(x, y, z) {
            this.append(Matrix3D.createTranslateMatrix(x, y, z));
        }


        // Returns a new Matrix3D object that is an exact copy of the current Matrix3D object.
        // return new Matrix3D
        clone() {
            return new Matrix3D(this._rawData);
        }

        // Converts the current matrix to an identity or unit matrix.
        // return void
        identity() {
            mat4.identity(this._rawData);
        }

        // [static] Simplifies the interpolation from one frame of reference to another by interpolating a display object a percent point closer to a target display object.
        // Matrix3D.interpolate = function() { };	

        // Interpolates the display object's matrix a percent closer to a target's matrix.
        // Matrix3D.prototype.interpolateTo = function() { };	


        // Inverts the current matrix.
        // return Boolean true if the matrix was successfully inverted.
        invert() {
            mat4.inverse(this._rawData);
        }


        // Rotates the display object so that it faces a specified position.
        // return void
        // Matrix3D.prototype.pointAt = function(pos, at, up)	{ };


        // Prepends a matrix by multiplying the current Matrix3D object by another Matrix3D object.
        // return void
        prepend(m) {
            mat4.multiply(m._rawData, this._rawData, this._rawData);
        }


        // Prepends an incremental scale change along the x, y, and z axes to a Matrix3D object.
        // return void
        prependScale(x, y, z) {
            this.prepend(Matrix3D.createScaleMatrix(x, y, z));
        }


        // Prepends an incremental translation, a repositioning along the x, y, and z axes, to a Matrix3D object.
        // return void
        prependTranslation(x, y, z) {
            this.prepend(Matrix3D.createTranslateMatrix(x, y, z));
        }


        // Uses the transformation matrix to transform a Vector3D object from one space coordinate to another.
        // return Vector3D with the transformed coordinates.
        transformVector(vector: Vector3D) {
            var vec = mat4.multiplyVec3(mat4.transpose(this._rawData, mat4.create()), [vector.x, vector.y, vector.z]);
            return new Vector3D(vec[0], vec[1], vec[2]);
        }


        // Converts the current Matrix3D object to a matrix where the rows and columns are swapped.
        transpose() {
            mat4.transpose(this._rawData);
        }


        static createTranslateMatrix(x, y, z) {
            return new Matrix3D([
                1, 0, 0, x,
                0, 1, 0, y,
                0, 0, 1, z,
                0, 0, 0, 1
            ]);
        }


        static createScaleMatrix(x, y, z) {
            return new Matrix3D([
                x, 0, 0, 0,
                0, y, 0, 0,
                0, 0, z, 0,
                0, 0, 0, 1
            ]);
        }
    }
}