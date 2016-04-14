
module jiglib {
    export class PhysicsState {
        position = new Vector3D(); // Vector3D
        orientation = new Matrix3D(); // Matrix3D
        linVelocity = new Vector3D(); // Vector3D
        rotVelocity = new Vector3D(); // Vector3D

        getOrientationCols() {

            return JMatrix3D.getCols(this.orientation);

        }
    }
}