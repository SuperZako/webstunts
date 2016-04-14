
/// <reference path="CollDetectFunctor.ts"/>

module jiglib {

    export class CollDetectCapsuleBox extends CollDetectFunctor {

        constructor() {
            super("CapsuleBox", "CAPSULE", "BOX");
        }

        collDetect(info, collArr) {
        }
    }
}