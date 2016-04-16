/// <reference path="../cof/JConfig.ts"/>

/// <reference path="../data/SpanData.ts"/>
/// <reference path="../data/EdgeData.ts"/>

/// <reference path="../geom/Vector3D.ts"/>
/// <reference path="../geom/Matrix3D.ts"/>

/// <reference path="../geometry/JAABox.ts"/>

/// <reference path="../math/JMath3D.ts"/>
/// <reference path="../math/JNumber3D.ts"/>
/// <reference path="../math/JMatrix3D.ts"/>

/// <reference path="MaterialProperties.ts"/>
/// <reference path="PhysicsState.ts"/>
/// <reference path="PhysicsSystem.ts"/>


module jiglib {
    export interface ISkin3D {

		/**
		 * @return A matrix with the current transformation values of the mesh.
		 */
        transform: JMatrix3D;


        vertices: Vector3D[];
        indices: TriangleVertexIndices[];
    }

    export class RigidBody {

        _id = null; // int
        _skin: ISkin3D = null; // ISkin3D
        _type = null; // String
        _boundingSphere: number = null; // Number
        _boundingBox = new JAABox(); // JAABox
        _currState = new PhysicsState();
        _oldState = new PhysicsState();
        _storeState = new PhysicsState();
        _invOrientation = null; // Matrix3D
        _currLinVelocityAux = null; // Vector3D
        _currRotVelocityAux = null; // Vector3D
        _mass = null; // Number
        _invMass = null; // Number
        _bodyInertia: Matrix3D = null; // Matrix3D
        _bodyInvInertia: Matrix3D = null; // Matrix3D
        _worldInertia: Matrix3D = null; // Matrix3D
        _worldInvInertia: Matrix3D = null; // Matrix3D
        _force = null; // Vector3D
        _torque = null; // Vector3D
        _linVelDamping = null; // Vector3D
        _rotVelDamping = null; // Vector3D
        _maxLinVelocities = null; // Vector3D
        _maxRotVelocities = null; // Vector3D
        _movable: boolean = null; // Boolean
        _origMovable: boolean = null; // Boolean
        _inactiveTime = null; // Number
        _bodiesToBeActivatedOnMovement = null; // RigidBody
        _storedPositionForActivation = null; // Vector3D
        _lastPositionForDeactivation = null; // Vector3D
        _lastOrientationForDeactivation = null; // Matrix3D
        _material = new MaterialProperties(); // MaterialProperties
        _rotationX = 0; // Number
        _rotationY = 0; // Number
        _rotationZ = 0; // Number
        _useDegrees = null; // Boolean
        _nonCollidables = null; // RigidBody
        _collideBodies = null; // RigidBody
        _constraints = null; // JConstraint
        _gravity = null; // Vector3D
        _gravityAxis = null; // int
        _gravityForce = null; // Vector3D
        collisions = null; // CollisionInfo
        externalData = null; // CollisionSystemGridEntry
        collisionSystem = null; // CollisionSystemAbstract
        isActive = null; // Boolean

        constructor(skin) {
            this._useDegrees = (JConfig.rotationType == "DEGREES") ? true : false;

            this._id = RigidBody.idCounter++;

            this._skin = skin;

            this._bodyInertia = new Matrix3D();
            this._bodyInvInertia = JMatrix3D.getInverseMatrix(this._bodyInertia);

            this._currLinVelocityAux = new Vector3D();
            this._currRotVelocityAux = new Vector3D();

            this._force = new Vector3D();
            this._torque = new Vector3D();

            this._invOrientation = JMatrix3D.getInverseMatrix(this._currState.orientation);
            this._linVelDamping = new Vector3D(0.999, 0.999, 0.999);
            this._rotVelDamping = new Vector3D(0.999, 0.999, 0.999);
            this._maxLinVelocities = new Vector3D(JMath3D.NUM_HUGE, JMath3D.NUM_HUGE, JMath3D.NUM_HUGE);
            this._maxRotVelocities = new Vector3D(JMath3D.NUM_HUGE, JMath3D.NUM_HUGE, JMath3D.NUM_HUGE);

            this._inactiveTime = 0;
            this.isActive = true;
            this._movable = true;
            this._origMovable = true;

            this.collisions = [];
            this._constraints = [];
            this._nonCollidables = [];
            this._collideBodies = [];

            this._storedPositionForActivation = new Vector3D();
            this._bodiesToBeActivatedOnMovement = [];
            this._lastPositionForDeactivation = this._currState.position.clone();
            this._lastOrientationForDeactivation = this._currState.orientation.clone();

            this._type = "Object3D";
            this._boundingSphere = JMath3D.NUM_HUGE;

            this.externalData = null;

        }


        updateRotationValues() {

            //var rotationVector = this._currState.orientation.decompose()[1];


        }

        get_rotationX() {

            return this._rotationX; //(this._useDegrees) ? this.radiansToDegrees(this._rotationX) : this._rotationX;

        }

        get_rotationY() {

            return this._rotationY; //(this._useDegrees) ? this.radiansToDegrees(this._rotationY) : this._rotationY;

        }

        get_rotationZ() {

            return this._rotationZ; //(this._useDegrees) ? this.radiansToDegrees(this._rotationZ) : this._rotationZ;

        }

        set_rotationX(px) {

            //var rad = (this._useDegrees) ? this.degreesToRadians(px) : px;
            this._rotationX = px;
            this.setOrientation(this.createRotationMatrix());

        }

        set_rotationY(py) {

            //var rad = (this._useDegrees) ? this.degreesToRadians(py) : py;
            this._rotationY = py;
            this.setOrientation(this.createRotationMatrix());

        }

        set_rotationZ(pz) {

            //var rad = (this._useDegrees) ? this.degreesToRadians(pz) : pz;
            this._rotationZ = pz;
            this.setOrientation(this.createRotationMatrix());

        }

        pitch(rot) {

            this.setOrientation(JMatrix3D.getAppendMatrix3D(this.get_currentState().orientation, JMatrix3D.getRotationMatrixAxis(rot, Vector3D.X_AXIS)));

        }

        yaw(rot) {

            this.setOrientation(JMatrix3D.getAppendMatrix3D(this.get_currentState().orientation, JMatrix3D.getRotationMatrixAxis(rot, Vector3D.Y_AXIS)));

        }

        roll(rot) {

            this.setOrientation(JMatrix3D.getAppendMatrix3D(this.get_currentState().orientation, JMatrix3D.getRotationMatrixAxis(rot, Vector3D.Z_AXIS)));

        }

        createRotationMatrix() {

            var matrix3D = new Matrix3D();
            matrix3D.appendRotation(this._rotationX, Vector3D.X_AXIS);
            matrix3D.appendRotation(this._rotationY, Vector3D.Y_AXIS);
            matrix3D.appendRotation(this._rotationZ, Vector3D.Z_AXIS);
            return matrix3D;

        }

        setOrientation(orient) {

            this._currState.orientation = orient.clone();
            this.updateInertia();
            this.updateState();

        }

        get_x() {

            return this._currState.position.x;

        }

        get_y() {

            return this._currState.position.y;

        }

        get_z() {

            return this._currState.position.z;

        }

        set_x(px) {

            this._currState.position.x = px;
            this.updateState();

        }

        set_y(py) {

            this._currState.position.y = py;
            this.updateState();

        }

        set_z(pz) {

            this._currState.position.z = pz;
            this.updateState();

        }

        moveTo(pos) {

            this._currState.position = pos.clone();
            this.updateState();

        }

        updateState() {

            this._currState.linVelocity.setTo(0, 0, 0);
            this._currState.rotVelocity.setTo(0, 0, 0);
            this.copyCurrentStateToOld();
            this.updateBoundingBox(); // todo: is making invalid boundingboxes, shouldn't this only be update when it's scaled?
            this.updateObject3D();

            if (this.collisionSystem) {
                this.collisionSystem.collisionSkinMoved(this);
            }

        }

        setLineVelocity(vel) {

            this._currState.linVelocity = vel.clone();

        }

        setAngleVelocity(angVel) {

            this._currState.rotVelocity = angVel.clone();

        }

        setLineVelocityAux(vel) {

            this._currLinVelocityAux = vel.clone();

        }

        setAngleVelocityAux(angVel) {

            this._currRotVelocityAux = angVel.clone();

        }

        updateGravity(gravity, gravityAxis) {

            this._gravity = gravity;
            this._gravityAxis = gravityAxis;

            this._gravityForce = JNumber3D.getScaleVector(this._gravity, this._mass);

        }

        addWorldTorque(t, active = null) {
            if (active == null) active = true;

            if (!this._movable) {
                return;
            }
            this._torque = this._torque.add(t);

            if (active) this.setActive();

        }

        addBodyTorque(t, active) {
            if (active == null) active = true;

            if (!this._movable)
                return;

            this.addWorldTorque(this._currState.orientation.transformVector(t), active);

        }

        addWorldForce(f, p, active) {
            if (active == null) active = true;

            if (!this._movable)
                return;

            this._force = this._force.add(f);
            this.addWorldTorque(p.subtract(this._currState.position).crossProduct(f));

            if (active) this.setActive();

        }

        addBodyForce(f, p, active = true) {
            //if (active == null) active = true;

            if (!this._movable)
                return;

            f = this._currState.orientation.transformVector(f);
            p = this._currState.orientation.transformVector(p);
            this.addWorldForce(f, this._currState.position.add(p), active);

        }

        clearForces() {

            this._force.setTo(0, 0, 0);
            this._torque.setTo(0, 0, 0);

        }

        applyWorldImpulse(impulse, pos, active = true) {
            //if (active == null) active = true;

            if (!this._movable) {
                return;
            }
            this._currState.linVelocity = this._currState.linVelocity.add(JNumber3D.getScaleVector(impulse, this._invMass));

            var rotImpulse = pos.subtract(this._currState.position).crossProduct(impulse);
            rotImpulse = this._worldInvInertia.transformVector(rotImpulse);
            this._currState.rotVelocity = this._currState.rotVelocity.add(rotImpulse);

            if (active) this.setActive();

        }

        applyWorldImpulseAux(impulse, pos, active = true) {
            //if (active == null) active = true;

            if (!this._movable) {
                return;
            }
            this._currLinVelocityAux = this._currLinVelocityAux.add(JNumber3D.getScaleVector(impulse, this._invMass));

            var rotImpulse = pos.subtract(this._currState.position).crossProduct(impulse);
            rotImpulse = this._worldInvInertia.transformVector(rotImpulse);
            this._currRotVelocityAux = this._currRotVelocityAux.add(rotImpulse);

            if (active) this.setActive();

        }

        applyBodyWorldImpulse(impulse, delta, active = true) {
            //if (active == null) active = true;

            if (!this._movable) {
                return;
            }
            this._currState.linVelocity = this._currState.linVelocity.add(JNumber3D.getScaleVector(impulse, this._invMass));

            var rotImpulse = delta.crossProduct(impulse);
            rotImpulse = this._worldInvInertia.transformVector(rotImpulse);
            this._currState.rotVelocity = this._currState.rotVelocity.add(rotImpulse);

            if (active)
                this.setActive();

        }

        applyBodyWorldImpulseAux(impulse, delta, active) {
            if (active == null) active = true;

            if (!this._movable) {
                return;
            }
            this._currLinVelocityAux = this._currLinVelocityAux.add(JNumber3D.getScaleVector(impulse, this._invMass));

            var rotImpulse = delta.crossProduct(impulse);
            rotImpulse = this._worldInvInertia.transformVector(rotImpulse);
            this._currRotVelocityAux = this._currRotVelocityAux.add(rotImpulse);

            if (active) this.setActive();

        }

        updateVelocity(dt) {

            if (!this._movable || !this.isActive)
                return;

            this._currState.linVelocity = this._currState.linVelocity.add(JNumber3D.getScaleVector(this._force, this._invMass * dt));

            var rac = JNumber3D.getScaleVector(this._torque, dt);
            rac = this._worldInvInertia.transformVector(rac);
            this._currState.rotVelocity = this._currState.rotVelocity.add(rac);

        }

        //updatePosition(dt) {

        //    if (!this._movable || !this.isActive) {
        //        return;
        //    }

        //    var angMomBefore = this._currState.rotVelocity.clone();
        //    angMomBefore = this._worldInertia.transformVector(angMomBefore);

        //    this._currState.position = this._currState.position.add(JNumber3D.getScaleVector(this._currState.linVelocity, dt));

        //    var dir = this._currState.rotVelocity.clone();
        //    var ang = dir.get_length();
        //    if (ang > 0) {
        //        dir.normalize();
        //        ang *= dt;
        //        var rot = JMatrix3D.rotationMatrix(dir.x, dir.y, dir.z, ang);
        //        this._currState.orientation = JMatrix3D.getMatrix3D(JMatrix3D.multiply(rot, JMatrix3D.getJMatrix3D(this._currState.orientation)));
        //        this.updateInertia();
        //    }

        //    angMomBefore = this._worldInvInertia.transformVector(angMomBefore);
        //    this._currState.rotVelocity = angMomBefore.clone();

        //}

        updatePositionWithAux(dt) {

            if (!this._movable || !this.isActive) {
                this._currLinVelocityAux.setTo(0, 0, 0);
                this._currRotVelocityAux.setTo(0, 0, 0);
                return;
            }

            var ga = this._gravityAxis;
            if (ga != -1) {
                var arr = JNumber3D.toArray(this._currLinVelocityAux);
                arr[(ga + 1) % 3] *= 0.1;
                arr[(ga + 2) % 3] *= 0.1;
                JNumber3D.copyFromArray(this._currLinVelocityAux, arr);
            }

            this._currState.position = this._currState.position.add(JNumber3D.getScaleVector(this._currState.linVelocity.add(this._currLinVelocityAux), dt));

            var dir = this._currState.rotVelocity.add(this._currRotVelocityAux);
            var ang = dir.get_length() * 180 / Math.PI;
            if (ang > 0) {
                dir.normalize();
                ang *= dt;


                var rot = JMatrix3D.getRotationMatrix(dir.x, dir.y, dir.z, ang);
                this._currState.orientation = JMatrix3D.getAppendMatrix3D(this._currState.orientation, rot);

                this.updateInertia();
            }

            this._currLinVelocityAux.setTo(0, 0, 0);
            this._currRotVelocityAux.setTo(0, 0, 0);


        }

        tryToFreeze(dt) {

            if (!this._movable || !this.isActive) {
                return;
            }

            if (this._currState.position.subtract(this._lastPositionForDeactivation).get_length() > JConfig.posThreshold) {
                this._lastPositionForDeactivation = this._currState.position.clone();
                this._inactiveTime = 0;
                return;
            }

            var ot = JConfig.orientThreshold;
            var deltaMat = JMatrix3D.getSubMatrix(this._currState.orientation, this._lastOrientationForDeactivation);

            var cols = JMatrix3D.getCols(deltaMat);

            if (cols[0].get_length() > ot || cols[1].get_length() > ot || cols[2].get_length() > ot) {
                this._lastOrientationForDeactivation = this._currState.orientation.clone();
                this._inactiveTime = 0;
                return;
            }

            if (this.getShouldBeActive()) {
                return;
            }

            this._inactiveTime += dt;
            if (this._inactiveTime > JConfig.deactivationTime) {
                this._lastPositionForDeactivation = this._currState.position.clone();
                this._lastOrientationForDeactivation = this._currState.orientation.clone();
                this.setInactive();
            }

        }

        postPhysics(dt) {

            if (!this._movable || !this.isActive) {
                return;
            }

            this.limitVel();
            this.limitAngVel();

            this.updatePositionWithAux(dt);
            this.updateBoundingBox(); // todo: is making invalid boundingboxes, shouldn't this only be update when it's scaled?
            this.updateObject3D();

            if (this.collisionSystem) {
                this.collisionSystem.collisionSkinMoved(this);
            }

            this.clearForces();

            //add gravity
            this._force = this._force.add(this._gravityForce);

        }

        set_mass(m) {

            this._mass = m;
            this._invMass = 1 / m;
            this.setInertia(this.getInertiaProperties(m));

            // this.get_mass() is dirty have to recalculate gravity this.get_force()
            var physicsSystem = PhysicsSystem.getInstance();
            this.updateGravity(physicsSystem.get_gravity(), physicsSystem.get_gravityAxis());

        }

        setInertia(matrix3D) {

            this._bodyInertia = matrix3D.clone();
            this._bodyInvInertia = JMatrix3D.getInverseMatrix(this._bodyInertia.clone());

            this.updateInertia();

        }

        updateInertia() {

            this._invOrientation = JMatrix3D.getTransposeMatrix(this._currState.orientation);

            this._worldInertia = JMatrix3D.getAppendMatrix3D(this._invOrientation, JMatrix3D.getAppendMatrix3D(this._currState.orientation, this._bodyInertia));

            this._worldInvInertia = JMatrix3D.getAppendMatrix3D(this._invOrientation, JMatrix3D.getAppendMatrix3D(this._currState.orientation, this._bodyInvInertia));

        }

        get_movable() {

            return this._movable;

        }

        set_movable(mov) {

            if (this._type == "PLANE" || this._type == "TERRAIN" || this._type == "TRIANGLEMESH")
                return;

            this._movable = mov;
            this.isActive = mov;
            this._origMovable = mov;

        }

        internalSetImmovable() {

            this._origMovable = this._movable;
            this._movable = false;

        }

        internalRestoreImmovable() {

            this._movable = this._origMovable;

        }

        setActive() {

            if (this._movable) {
                if (this.isActive) return;
                this._inactiveTime = 0;
                this.isActive = true;
            }

        }

        setInactive() {

            if (this._movable) {
                this._inactiveTime = JConfig.deactivationTime;
                this.isActive = false;
            }

        }

        getVelocity(relPos) {

            return this._currState.linVelocity.add(this._currState.rotVelocity.crossProduct(relPos));

        }

        getVelocityAux(relPos) {

            return this._currLinVelocityAux.add(this._currRotVelocityAux.crossProduct(relPos));

        }

        getShouldBeActive() {

            return ((this._currState.linVelocity.get_length() > JConfig.velThreshold) || (this._currState.rotVelocity.get_length() > JConfig.angVelThreshold));

        }

        dampForDeactivation() {

            this._currState.linVelocity.x *= this._linVelDamping.x;
            this._currState.linVelocity.y *= this._linVelDamping.y;
            this._currState.linVelocity.z *= this._linVelDamping.z;
            this._currState.rotVelocity.x *= this._rotVelDamping.x;
            this._currState.rotVelocity.y *= this._rotVelDamping.y;
            this._currState.rotVelocity.z *= this._rotVelDamping.z;

            this._currLinVelocityAux.x *= this._linVelDamping.x;
            this._currLinVelocityAux.y *= this._linVelDamping.y;
            this._currLinVelocityAux.z *= this._linVelDamping.z;
            this._currRotVelocityAux.x *= this._rotVelDamping.x;
            this._currRotVelocityAux.y *= this._rotVelDamping.y;
            this._currRotVelocityAux.z *= this._rotVelDamping.z;

            var r = 0.5;
            var frac = this._inactiveTime / JConfig.deactivationTime;
            if (frac < r)
                return;

            var scale = 1 - ((frac - r) / (1 - r));
            if (scale < 0) {
                scale = 0;
            }
            else if (scale > 1) {
                scale = 1;
            }

            this._currState.linVelocity.scaleBy(scale);
            this._currState.rotVelocity.scaleBy(scale);

        }

        doMovementActivations(physicsSystem) {

            if (this._bodiesToBeActivatedOnMovement.length == 0 || this._currState.position.subtract(this._storedPositionForActivation).get_length() < JConfig.posThreshold)
                return;

            for (var _bodiesToBeActivatedOnMovement_i = 0, _bodiesToBeActivatedOnMovement_l = this._bodiesToBeActivatedOnMovement.length, body; (_bodiesToBeActivatedOnMovement_i < _bodiesToBeActivatedOnMovement_l) && (body = this._bodiesToBeActivatedOnMovement[_bodiesToBeActivatedOnMovement_i]); _bodiesToBeActivatedOnMovement_i++) {
                physicsSystem.activateObject(body);
            }

            this._bodiesToBeActivatedOnMovement.length = 0;

        }

        addMovementActivation(pos, otherBody) {

            if (this._bodiesToBeActivatedOnMovement.indexOf(otherBody) > -1)
                return;

            if (this._bodiesToBeActivatedOnMovement.length == 0)
                this._storedPositionForActivation = pos;

            this._bodiesToBeActivatedOnMovement.push(otherBody);

        }

        setConstraintsAndCollisionsUnsatisfied() {

            for (var _constraints_i = 0, _constraints_l = this._constraints.length, _constraint; (_constraints_i < _constraints_l) && (_constraint = this._constraints[_constraints_i]); _constraints_i++)
                _constraint.satisfied = false;

            for (var collisions_i = 0, collisions_l = this.collisions.length, _collision; (collisions_i < collisions_l) && (_collision = this.collisions[collisions_i]); collisions_i++)
                _collision.satisfied = false;

        }

        segmentIntersect(out: CollOutBodyData, seg: JSegment, state) {

            return false;

        }

        getInertiaProperties(m) {

            return new Matrix3D();

        }

        updateBoundingBox() {


        }

        hitTestObject3D(obj3D) {

            var num1, num2;
            num1 = this._currState.position.subtract(obj3D.get_currentState().position).get_length();
            num2 = this._boundingSphere + obj3D.get_boundingSphere();

            return num1 <= num2;


        }

        disableCollisions(body) {

            if (this._nonCollidables.indexOf(body) < 0)
                this._nonCollidables.push(body);

        }

        enableCollisions(body) {

            if (this._nonCollidables.indexOf(body) >= 0)
                this._nonCollidables.splice(this._nonCollidables.indexOf(body), 1);

        }

        addCollideBody(body) {

            if (this._collideBodies.indexOf(body) < 0) {

                this._collideBodies.push(body);


            }

        }

        removeCollideBodies(body) {

            var i = this._collideBodies.indexOf(body);
            if (i >= 0) {

                this._collideBodies.splice(i, 1);


            }

        }

        addConstraint(constraint) {

            if (this._constraints.indexOf(constraint) < 0) {
                this._constraints.push(constraint);
            }

        }

        removeConstraint(constraint) {

            if (this._constraints.indexOf(constraint) >= 0) {
                this._constraints.splice(this._constraints.indexOf(constraint), 1);
            }

        }

        copyCurrentStateToOld() {

            this._oldState.position = this._currState.position.clone();
            this._oldState.orientation = this._currState.orientation.clone();
            this._oldState.linVelocity = this._currState.linVelocity.clone();
            this._oldState.rotVelocity = this._currState.rotVelocity.clone();

        }

        storeState() {

            this._storeState.position = this._currState.position.clone();
            this._storeState.orientation = this._currState.orientation.clone();
            this._storeState.linVelocity = this._currState.linVelocity.clone();
            this._storeState.rotVelocity = this._currState.rotVelocity.clone();

        }

        restoreState() {

            this._currState.position = this._storeState.position.clone();
            this._currState.orientation = this._storeState.orientation.clone();
            this._currState.linVelocity = this._storeState.linVelocity.clone();
            this._currState.rotVelocity = this._storeState.rotVelocity.clone();

            this.updateInertia();

        }

        get_currentState() {

            return this._currState;

        }

        get_oldState() {

            return this._oldState;

        }

        get_id() {

            return this._id;

        }

        get_type() {

            return this._type;

        }

        get_skin() {

            return this._skin;

        }

        get_boundingSphere() {

            return this._boundingSphere;

        }

        get_boundingBox() {

            return this._boundingBox;

        }

        get_force() {

            return this._force;

        }

        get_mass() {

            return this._mass;

        }

        get_invMass() {

            return this._invMass;

        }

        get_worldInertia() {

            return this._worldInertia;

        }

        get_worldInvInertia() {

            return this._worldInvInertia;

        }

        get_nonCollidables() {

            return this._nonCollidables;

        }

        get_constraints() {

            return this._constraints;

        }

        set_linVelocityDamping(vel) {

            this._linVelDamping.x = JMath3D.getLimiteNumber(vel.x, 0, 1);
            this._linVelDamping.y = JMath3D.getLimiteNumber(vel.y, 0, 1);
            this._linVelDamping.z = JMath3D.getLimiteNumber(vel.z, 0, 1);

        }

        get_linVelocityDamping() {

            return this._linVelDamping;

        }

        set_rotVelocityDamping(vel) {

            this._rotVelDamping.x = JMath3D.getLimiteNumber(vel.x, 0, 1);
            this._rotVelDamping.y = JMath3D.getLimiteNumber(vel.y, 0, 1);
            this._rotVelDamping.z = JMath3D.getLimiteNumber(vel.z, 0, 1);

        }

        get_rotVelocityDamping() {

            return this._rotVelDamping;

        }

        set_maxLinVelocities(vel) {

            this._maxLinVelocities = new Vector3D(Math.abs(vel.x), Math.abs(vel.y), Math.abs(vel.z));

        }

        get_maxLinVelocities() {

            return this._maxLinVelocities;

        }

        get_maxRotVelocities() {

            return this._maxRotVelocities;

        }

        limitVel() {

            this._currState.linVelocity.x = JMath3D.getLimiteNumber(this._currState.linVelocity.x, -this._maxLinVelocities.x, this._maxLinVelocities.x);
            this._currState.linVelocity.y = JMath3D.getLimiteNumber(this._currState.linVelocity.y, -this._maxLinVelocities.y, this._maxLinVelocities.y);
            this._currState.linVelocity.z = JMath3D.getLimiteNumber(this._currState.linVelocity.z, -this._maxLinVelocities.z, this._maxLinVelocities.z);

        }

        limitAngVel() {

            var fx = Math.abs(this._currState.rotVelocity.x) / this._maxRotVelocities.x;
            var fy = Math.abs(this._currState.rotVelocity.y) / this._maxRotVelocities.y;
            var fz = Math.abs(this._currState.rotVelocity.z) / this._maxRotVelocities.z;
            var f = Math.max(fx, fy, fz);

            if (f > 1)
                this._currState.rotVelocity = JNumber3D.getDivideVector(this._currState.rotVelocity, f);

        }

        getTransform() {

            return this._skin ? this._skin.transform : null;

        }

        updateObject3D() {

            if (this._skin) {
                this._skin.transform = JMatrix3D.getAppendMatrix3D(this._currState.orientation, JMatrix3D.getTranslationMatrix(this._currState.position.x, this._currState.position.y, this._currState.position.z));
            }

        }

        get_material() {

            return this._material;

        }

        get_restitution() {

            return this._material.restitution;

        }

        set_restitution(restitution) {

            this._material.restitution = JMath3D.getLimiteNumber(restitution, 0, 1);

        }

        get_friction() {

            return this._material.friction;

        }

        set_friction(friction) {

            this._material.friction = JMath3D.getLimiteNumber(friction, 0, 1);

        }

        static idCounter = 0; // int

        static formatRotation(angle) {

            if (angle >= -180 && angle <= 180)
                return angle;

            var angle2 = angle % 360;
            if (angle2 < -180)
                return angle2 + 360;

            if (angle2 > 180)
                return angle2 - 360;

            return angle2;
        }
    }
}