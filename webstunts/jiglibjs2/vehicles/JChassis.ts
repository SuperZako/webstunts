
/// <reference path="../geometry/JBox.ts"/>

module jiglib {

    export class JChassis extends JBox {
        _car = null; // JCar
        constructor(car: JCar, skin = null, width = null, depth = null, height = null) {
            super(skin, width, depth, height);
            this._car = car;
        }

        get_car() {
            return this._car;
        }

        postPhysics(dt) {
            super.postPhysics(dt);
            this._car.addExternalForces(dt);
            this._car.postPhysics(dt);

        }
    }
}