

/// <reference path="CollOutData.ts"/>


module jiglib {

    export class CollOutBodyData extends CollOutData {
        rigidBody = null; // RigidBody
        constructor(frac = null, position = null, normal = null, rigidBody = null) {
            super(frac, position, normal);
            this.rigidBody = rigidBody;
        }
    }
}