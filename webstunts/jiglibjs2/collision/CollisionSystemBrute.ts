/// <reference path="CollisionSystemAbstract.ts"/>

module jiglib {
    export class CollisionSystemBrute extends CollisionSystemAbstract {

        constructor() {
            super();
        }

        detectAllCollisions(bodies: RigidBody[], collArr) {
            this._numCollisionsChecks = 0;
            for (let _body of bodies) {

                if (!_body.isActive)
                    continue;

                let bodyID = _body.get_id();
                let bodyType = _body.get_type();

                for (let _collBody of this.collBody) {
                    if (_body == _collBody) {
                        continue;
                    }

                    if (_collBody.isActive && bodyID > _collBody.get_id()) {
                        continue;
                    }

                    if (this.checkCollidables(_body, _collBody) && this.detectionFunctors[bodyType + "_" + _collBody.get_type()] != undefined) {
                        let info = new CollDetectInfo();
                        info.body0 = _body;
                        info.body1 = _collBody;
                        let fu = this.detectionFunctors[info.body0.get_type() + "_" + info.body1.get_type()];
                        fu.collDetect(info, collArr);
                        this._numCollisionsChecks += 1;
                    }
                }
            }
        }
    }
}