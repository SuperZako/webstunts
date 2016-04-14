
/// <reference path="../geom/Vector3D.ts"/>
namespace jiglib {
    export class CollOutData {
        constructor(public frac = 0, public position = new Vector3D, public normal = new Vector3D) {
        }
    }
}