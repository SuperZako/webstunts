

/// <reference path="../cof/JConfig.ts"/>

/// <reference path="../data/SpanData.ts"/>

/// <reference path="../geom/Vector3D.ts"/>

/// <reference path="../math/JMath3D.ts"/>
/// <reference path="../math/JNumber3D.ts"/>

/// <reference path="../physics/MaterialProperties.ts"/>
/// <reference path="../physics/RigidBody.ts"/>

/// <reference path="JChassis.ts"/>
/// <reference path="JWheel.ts"/>


module jiglib {



    export class JCar {
        _maxSteerAngle = null; // Number
        _steerRate = null; // Number
        _driveTorque = null; // Number
        _destSteering = null; // Number
        _destAccelerate = null; // Number
        _steering = null; // Number
        _accelerate = null; // Number
        _HBrake = null; // Number
        _chassis: JChassis = null; // JChassis
        _wheels = null; // Array
        _steerWheels = null; // Array
        constructor(skin) {

            this._chassis = new JChassis(this, skin);
            this._wheels = [];
            this._steerWheels = [];
            this._destSteering = this._destAccelerate = this._steering = this._accelerate = this._HBrake = 0;
            this.setCar();

        }

        setCar(maxSteerAngle= null, steerRate= null, driveTorque= null) {
            if (maxSteerAngle == null) maxSteerAngle = 45;
            if (steerRate == null) steerRate = 1;
            if (driveTorque == null) driveTorque = 500;

            this._maxSteerAngle = maxSteerAngle;
            this._steerRate = steerRate;
            this._driveTorque = driveTorque;

        }

        setupWheel(_name, pos, wheelSideFriction, wheelFwdFriction, wheelTravel, wheelRadius, wheelRestingFrac, wheelDampingFrac, wheelNumRays) {
            if (wheelSideFriction == null) wheelSideFriction = 2;
            if (wheelFwdFriction == null) wheelFwdFriction = 2;
            if (wheelTravel == null) wheelTravel = 3;
            if (wheelRadius == null) wheelRadius = 10;
            if (wheelRestingFrac == null) wheelRestingFrac = 0.5;
            if (wheelDampingFrac == null) wheelDampingFrac = 0.5;
            if (wheelNumRays == null) wheelNumRays = 1;

            var mass = this._chassis.get_mass();
            var mass4 = 0.25 * mass;

            var gravity = PhysicsSystem.getInstance().get_gravity().clone();
            var gravityLen = PhysicsSystem.getInstance().get_gravity().get_length();
            gravity.normalize();
            var axis = JNumber3D.getScaleVector(gravity, -1);
            var spring = mass4 * gravityLen / (wheelRestingFrac * wheelTravel);
            var inertia = 0.015 * wheelRadius * wheelRadius * mass;
            var damping = 2 * Math.sqrt(spring * mass);
            damping *= (0.25 * wheelDampingFrac);

            this._wheels[_name] = new JWheel(this);
            this._wheels[_name].setup(pos, axis, spring, wheelTravel, inertia,
                wheelRadius, wheelSideFriction, wheelFwdFriction,
                damping, wheelNumRays);

        }

        get_chassis() {

            return this._chassis;

        }
        get_wheels() {

            return this._wheels;

        }

        setAccelerate(val) {

            this._destAccelerate = val;

        }

        setSteer(wheels, val) {

            this._destSteering = val;
            this._steerWheels = [];
            for (var i in wheels) {
                if (this.findWheel(wheels[i])) {
                    this._steerWheels[wheels[i]] = this._wheels[wheels[i]];
                }
            }

        }

        findWheel(_name) {

            for (var i in this._wheels) {
                if (i == _name) {
                    return true;
                }
            }
            return false;

        }

        setHBrake(val) {

            this._HBrake = val;

        }

        addExternalForces(dt) {

            for (var wheels_i = 0, wheels_l = this.get_wheels().length, wheel; (wheels_i < wheels_l) && (wheel = this.get_wheels()[wheels_i]); wheels_i++) {
                wheel.addForcesToCar(dt);
            }

        }

        postPhysics(dt) {

            var wheel;
            for (var wheels_i = 0, wheels_l = this.get_wheels().length, wheel; (wheels_i < wheels_l) && (wheel = this.get_wheels()[wheels_i]); wheels_i++) {
                wheel.update(dt);
            }

            var deltaAccelerate, deltaSteering, dAccelerate, dSteering, alpha, angleSgn;
            deltaAccelerate = dt;
            deltaSteering = dt * this._steerRate;
            dAccelerate = this._destAccelerate - this._accelerate;
            if (dAccelerate < -deltaAccelerate) {
                dAccelerate = -deltaAccelerate;
            }
            else if (dAccelerate > deltaAccelerate) {
                dAccelerate = deltaAccelerate;
            }
            this._accelerate += dAccelerate;

            dSteering = this._destSteering - this._steering;
            if (dSteering < -deltaSteering) {
                dSteering = -deltaSteering;
            }
            else if (dSteering > deltaSteering) {
                dSteering = deltaSteering;
            }
            this._steering += dSteering;

            for (var wheels_i = 0, wheels_l = this.get_wheels().length, wheel; (wheels_i < wheels_l) && (wheel = this.get_wheels()[wheels_i]); wheels_i++) {
                wheel.addTorque(this._driveTorque * this._accelerate);
                wheel.setLock(this._HBrake > 0.5);
            }

            alpha = Math.abs(this._maxSteerAngle * this._steering);
            angleSgn = (this._steering > 0) ? 1 : -1;
            for (var _steerWheels_i = 0, _steerWheels_l = this._steerWheels.length, _steerWheel; (_steerWheels_i < _steerWheels_l) && (_steerWheel = this._steerWheels[_steerWheels_i]); _steerWheels_i++) {
                _steerWheel.setSteerAngle(angleSgn * alpha);
            }

        }

        getNumWheelsOnFloor() {
            var count = 0;
            for (var wheels_i = 0, wheels_l = this.get_wheels().length, wheel; (wheels_i < wheels_l) && (wheel = this.get_wheels()[wheels_i]); wheels_i++) {
                if (wheel.getOnFloor()) {
                    count++;
                }
            }
            return count;

        }

    }
}
