
/// <reference path="../cof/JConfig.ts"/>

/// <reference path="../data/SpanData.ts"/>
/// <reference path="../data/EdgeData.ts"/>

/// <reference path="../geom/Vector3D.ts"/>
/// <reference path="../geom/Matrix3D.ts"/>

/// <reference path="../geometry/JAABox.ts"/>

/// <reference path="../math/JMath3D.ts"/>
/// <reference path="../math/JNumber3D.ts"/>
/// <reference path="../math/JMatrix3D.ts"/>



module jiglib {
    export class JWheel {
        noslipVel = 0.2; // Number
        slipVel = 0.4; // Number
        slipFactor = 0.7; // Number
        smallVel = 3; // Number
        _car: JCar = null; // JCar
        _pos = null; // Vector3D
        _axisUp = null; // Vector3D
        _spring = null; // Number
        _travel = null; // Number
        _inertia = null; // Number
        _radius = null; // Number
        _sideFriction = null; // Number
        _fwdFriction = null; // Number
        _damping = null; // Number
        _numRays = null; // int
        _angVel = null; // Number
        _steerAngle = null; // Number
        _torque = null; // Number
        _driveTorque = null; // Number
        _axisAngle = null; // Number
        _displacement = null; // Number
        _upSpeed = null; // Number
        _rotDamping = null; // Number
        _locked = null; // Boolean
        _lastDisplacement = null; // Number
        _lastOnFloor = null; // Boolean
        _angVelForGrip = null; // Number
        worldPos = null; // Vector3D
        worldAxis = null; // Vector3D
        wheelFwd = null; // Vector3D
        wheelUp = null; // Vector3D
        wheelLeft = null; // Vector3D
        wheelRayEnd = null; // Vector3D
        wheelRay = null; // JSegment
        groundUp = null; // Vector3D
        groundLeft = null; // Vector3D
        groundFwd = null; // Vector3D
        wheelPointVel = null; // Vector3D
        rimVel = null; // Vector3D
        worldVel = null; // Vector3D
        wheelCentreVel = null; // Vector3D
        _collisionSystem: CollisionSystemAbstract = null; // CollisionSystemAbstract
        constructor(car) {
            this._car = car;

        }

        setup(pos, axisUp, spring, travel, inertia, radius, sideFriction, fwdFriction, damping, numRays) {

            this._pos = pos;
            this._axisUp = axisUp;
            this._spring = spring;
            this._travel = travel;
            this._inertia = inertia;
            this._radius = radius;
            this._sideFriction = sideFriction;
            this._fwdFriction = fwdFriction;
            this._damping = damping;
            this._numRays = numRays;
            this.reset();

        }

        addTorque(torque) {

            this._driveTorque += torque;

        }

        setLock(lock) {

            this._locked = lock;

        }

        setSteerAngle(steer) {

            this._steerAngle = steer;

        }

        getSteerAngle() {

            return this._steerAngle;

        }

        getPos() {

            return this._pos;

        }

        getLocalAxisUp() {

            return this._axisUp;

        }

        getActualPos() {

            return this._pos.add(JNumber3D.getScaleVector(this._axisUp, this._displacement));

        }

        getRadius() {

            return this._radius;

        }

        getDisplacement() {

            return this._displacement;

        }

        getAxisAngle() {

            return this._axisAngle;

        }

        getRollAngle() {

            return 0.1 * this._angVel * 180 / Math.PI;

        }

        setRotationDamping(vel) {

            this._rotDamping = vel;

        }

        getRotationDamping() {

            return this._rotDamping;

        }

        getOnFloor() {

            return this._lastOnFloor;

        }

        addForcesToCar(dt) {

            var force = new Vector3D();
            this._lastDisplacement = this._displacement;
            this._displacement = 0;

            var carBody = this._car.get_chassis();
            this.worldPos = carBody.get_currentState().orientation.transformVector(this._pos);
            this.worldPos = carBody.get_currentState().position.add(this.worldPos);
            this.worldAxis = carBody.get_currentState().orientation.transformVector(this._axisUp);

            this.wheelFwd = JMatrix3D.getRotationMatrix(this.worldAxis.x, this.worldAxis.y, this.worldAxis.z, this._steerAngle).transformVector(carBody.get_currentState().getOrientationCols()[2]);
            this.wheelUp = this.worldAxis;
            this.wheelLeft = this.wheelUp.crossProduct(this.wheelFwd);
            this.wheelLeft.normalize();

            var rayLen = 2 * this._radius + this._travel;
            this.wheelRayEnd = this.worldPos.subtract(JNumber3D.getScaleVector(this.worldAxis, this._radius));
            this.wheelRay = new JSegment(this.wheelRayEnd.add(JNumber3D.getScaleVector(this.worldAxis, rayLen)), JNumber3D.getScaleVector(this.worldAxis, -rayLen));

            if (!this._collisionSystem)
                this._collisionSystem = PhysicsSystem.getInstance().getCollisionSystem();

            var maxNumRays = 10;
            var numRays = Math.min(this._numRays, maxNumRays);

            var objArr = [];
            var segments = [];

            var deltaFwd = (2 * this._radius) / (numRays + 1);
            var deltaFwdStart = deltaFwd;

            this._lastOnFloor = false;

            var distFwd;
            var yOffset;
            var bestIRay = 0;
            var iRay = 0;
            var collOutBodyData;
            var segment;
            for (iRay = 0; iRay < numRays; iRay++) {
                collOutBodyData = objArr[iRay] = new CollOutBodyData();
                distFwd = (deltaFwdStart + iRay * deltaFwd) - this._radius;
                yOffset = this._radius * (1 - Math.cos(90 * (distFwd / this._radius) * Math.PI / 180));
                segment = segments[iRay] = this.wheelRay.clone();
                segment.origin = segment.origin.add(JNumber3D.getScaleVector(this.wheelFwd, distFwd).add(JNumber3D.getScaleVector(this.wheelUp, yOffset)));
                if (this._collisionSystem.segmentIntersect(collOutBodyData, segment, carBody)) {
                    this._lastOnFloor = true;
                    if (collOutBodyData.frac < objArr[bestIRay].frac) {
                        bestIRay = iRay;
                    }
                }
            }

            if (!this._lastOnFloor) {
                return false;
            }

            var frac = objArr[bestIRay].frac;
            var groundPos = objArr[bestIRay].position;
            var otherBody = objArr[bestIRay].rigidBody;

            var groundNormal = this.worldAxis.clone();
            if (numRays > 1) {
                for (iRay = 0; iRay < numRays; iRay++) {
                    collOutBodyData = objArr[iRay];
                    if (collOutBodyData.frac <= 1)
                        groundNormal = groundNormal.add(JNumber3D.getScaleVector(this.worldPos.subtract(segments[iRay].getEnd()), 1 - collOutBodyData.frac));
                }

                groundNormal.normalize();
            }
            else {
                groundNormal = objArr[bestIRay].normal;
            }

            this._displacement = rayLen * (1 - frac);

            if (this._displacement < 0)
                this._displacement = 0;
            else if (this._displacement > this._travel)
                this._displacement = this._travel;

            var displacementForceMag = this._displacement * this._spring;
            displacementForceMag *= groundNormal.dotProduct(this.worldAxis);

            var dampingForceMag = this._upSpeed * this._damping;
            var totalForceMag = displacementForceMag + dampingForceMag;
            if (totalForceMag < 0)
                totalForceMag = 0;

            var extraForce = JNumber3D.getScaleVector(this.worldAxis, totalForceMag);
            force = force.add(extraForce);

            this.groundUp = groundNormal;
            this.groundLeft = groundNormal.crossProduct(this.wheelFwd);
            this.groundLeft.normalize();
            this.groundFwd = this.groundLeft.crossProduct(this.groundUp);

            var tempv = carBody.get_currentState().orientation.transformVector(this._pos);
            this.wheelPointVel = carBody.get_currentState().linVelocity.add(carBody.get_currentState().rotVelocity.crossProduct(tempv));

            this.rimVel = JNumber3D.getScaleVector(this.wheelLeft.crossProduct(groundPos.subtract(this.worldPos)), this._angVel);
            this.wheelPointVel = this.wheelPointVel.add(this.rimVel);

            if (otherBody.get_movable()) {
                this.worldVel = otherBody.get_currentState().linVelocity.add(otherBody.get_currentState().rotVelocity.crossProduct(groundPos.subtract(otherBody.get_currentState().position)));
                this.wheelPointVel = this.wheelPointVel.subtract(this.worldVel);
            }

            var friction = this._sideFriction;
            var sideVel = this.wheelPointVel.dotProduct(this.groundLeft);

            if ((sideVel > this.slipVel) || (sideVel < -this.slipVel))
                friction *= this.slipFactor;
            else if ((sideVel > this.noslipVel) || (sideVel < -this.noslipVel))
                friction *= (1 - (1 - this.slipFactor) * (Math.abs(sideVel) - this.noslipVel) / (this.slipVel - this.noslipVel));

            if (sideVel < 0) {
                friction *= -1;
            }
            if (Math.abs(sideVel) < this.smallVel) {
                friction *= Math.abs(sideVel) / this.smallVel;
            }

            var sideForce = -friction * totalForceMag;
            extraForce = JNumber3D.getScaleVector(this.groundLeft, sideForce);
            force = force.add(extraForce);

            friction = this._fwdFriction;
            var fwdVel = this.wheelPointVel.dotProduct(this.groundFwd);
            if ((fwdVel > this.slipVel) || (fwdVel < -this.slipVel)) {
                friction *= this.slipFactor;
            }
            else if ((fwdVel > this.noslipVel) || (fwdVel < -this.noslipVel)) {
                friction *= (1 - (1 - this.slipFactor) * (Math.abs(fwdVel) - this.noslipVel) / (this.slipVel - this.noslipVel));
            }
            if (fwdVel < 0) {
                friction *= -1;
            }
            if (Math.abs(fwdVel) < this.smallVel) {
                friction *= (Math.abs(fwdVel) / this.smallVel);
            }
            var fwdForce = -friction * totalForceMag;
            extraForce = JNumber3D.getScaleVector(this.groundFwd, fwdForce);
            force = force.add(extraForce);

            this.wheelCentreVel = carBody.get_currentState().linVelocity.add(carBody.get_currentState().rotVelocity.crossProduct(tempv));
            this._angVelForGrip = this.wheelCentreVel.dotProduct(this.groundFwd) / this._radius;
            this._torque += (-fwdForce * this._radius);

            carBody.addWorldForce(force, groundPos, false);
            if (otherBody.get_movable()) {
                var maxOtherBodyAcc = 500;
                var maxOtherBodyForce = maxOtherBodyAcc * otherBody.get_mass();
                if (force.get_lengthSquared() > maxOtherBodyForce * maxOtherBodyForce) {
                    force = JNumber3D.getScaleVector(force, maxOtherBodyForce / force.get_length());
                }
                otherBody.addWorldForce(JNumber3D.getScaleVector(force, -1), groundPos, false);
            }
            return true;

        }

        update(dt) {

            if (dt <= 0) {
                return;
            }
            var origAngVel = this._angVel;
            this._upSpeed = (this._displacement - this._lastDisplacement) / Math.max(dt, JMath3D.NUM_TINY);

            if (this._locked) {
                this._angVel = 0;
                this._torque = 0;
            }
            else {
                this._angVel += (this._torque * dt / this._inertia);
                this._torque = 0;

                if (((origAngVel > this._angVelForGrip) && (this._angVel < this._angVelForGrip)) || ((origAngVel < this._angVelForGrip) && (this._angVel > this._angVelForGrip))) {
                    this._angVel = this._angVelForGrip;
                }

                this._angVel += this._driveTorque * dt / this._inertia;
                this._driveTorque = 0;

                if (this._angVel < -100) {
                    this._angVel = -100;
                }
                else if (this._angVel > 100) {
                    this._angVel = 100;
                }
                this._angVel *= this._rotDamping;
                this._axisAngle += (this._angVel * dt * 180 / Math.PI);
            }


        }

        reset() {

            this._angVel = 0;
            this._steerAngle = 0;
            this._torque = 0;
            this._driveTorque = 0;
            this._axisAngle = 0;
            this._displacement = 0;
            this._upSpeed = 0;
            this._locked = false;
            this._lastDisplacement = 0;
            this._lastOnFloor = false;
            this._angVelForGrip = 0;
            this._rotDamping = 0.99;

        }


    }
}