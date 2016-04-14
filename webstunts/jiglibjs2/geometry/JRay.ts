
/// <reference path="../math/JNumber3D.ts"/>

module jiglib {

    export class JRay {

        constructor(public origin, public dir) {
        }

        getOrigin(t) {

            return this.origin.add(JNumber3D.getScaleVector(this.dir, t));

        }

    }
}