
/// <reference path="../math/JNumber3D.ts"/>

module jiglib {

    export class JRay {

        constructor(public origin: Vector3D, public dir: Vector3D) {
        }

        getOrigin(t: number) {
            return this.origin.add(JNumber3D.getScaleVector(this.dir, t));
        }

    }
}