
/// <reference path="RigidBody.ts"/>


module jiglib {

    export class BodyPair {
        body0: RigidBody= null; // RigidBody
        body1: RigidBody= null; // RigidBody
        r = null; // Vector3D
        constructor(_body0, _body1, r0, r1) {


            var id1 = -1;
            if (_body1 != null)
                id1 = _body1.get_id();

            if (_body0.get_id() > id1) {
                this.body0 = _body0;
                this.body1 = _body1;
                this.r = r0;
            }
            else {
                this.body0 = _body1;
                this.body1 = _body0;
                this.r = r1;
            }

        }
    }
}