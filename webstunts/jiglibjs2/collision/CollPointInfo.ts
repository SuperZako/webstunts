

/// <reference path="../geom/Vector3D.ts"/>

module jiglib {



    export class CollPointInfo {
        initialPenetration :number= null; // Number
        r0: Vector3D= null; // Vector3D
        r1: Vector3D= null; // Vector3D
        position: Vector3D = null; // Vector3D
        minSeparationVel= 0; // Number
        denominator = 0; // Number
        accumulatedNormalImpulse = 0; // Number
        accumulatedNormalImpulseAux = 0; // Number
        accumulatedFrictionImpulse = new Vector3D(); // Vector3D
    }

}
