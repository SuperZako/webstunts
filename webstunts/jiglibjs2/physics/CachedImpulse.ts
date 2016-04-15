
module jiglib {
    export class CachedImpulse {
        constructor(public normalImpulse: number, public normalImpulseAux: number, public frictionImpulse: Vector3D) { }
    }
}