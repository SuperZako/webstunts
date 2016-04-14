

/// <reference path="CollOutData.ts"/>

namespace jiglib {
    export class CollOutBodyData extends CollOutData {
        constructor(frac = 0, position = new Vector3D(), normal = new Vector3D(), public rigidBody: RigidBody = null) {
            super(frac, position, normal);
        }
    }
}