
/// <reference path="../geom/Vector3D.ts"/>

module jiglib {

    export class CollOutData {
        frac :number= null; // Number
        position :Vector3D= null; // Vector3D
        normal:Vector3D = null; // Vector3D
        constructor(frac= null, position= null, normal= null) {
            this.frac = isNaN(frac) ? 0 : frac;
            this.position = position ? position : new Vector3D;
            this.normal = normal ? normal : new Vector3D;

        }
    }
}