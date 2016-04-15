/// <reference path="../cof/JConfig.ts"/>
/// <reference path="../collision/CollisionSystemGrid.ts"/>
/// <reference path="../collision/CollisionSystemBrute.ts"/>
/// <reference path="../data/ContactData.ts"/>
/// <reference path="BodyPair.ts"/>
/// <reference path="CachedImpulse.ts"/>

namespace jiglib {
    export class PhysicsSystem {
        private _maxVelMag = 0.5; // Number
        private _minVelForProcessing = 0.001; // Number
        private _bodies: RigidBody[] = [];//null; // RigidBody
        private _activeBodies: RigidBody[] = null; // RigidBody
        private _collisions: CollisionInfo[] = null; // CollisionInfo
        private _constraints = null; // JConstraint
        private _controllers: PhysicsController[] = null; // PhysicsController
        private _gravityAxis = null; // int
        private _gravity = null; // Vector3D
        private _doingIntegration = null; // Boolean
        preProcessCollisionFn = null; // Function
        preProcessContactFn = null; // Function
        processCollisionFn = null; // Function
        processContactFn = null; // Function
        private _cachedContacts: ContactData[] = null; // ContactData
        private _collisionSystem: CollisionSystemAbstract = null; // CollisionSystemAbstract

        constructor() {
            this.setSolverType(JConfig.solverType);
            this._doingIntegration = false;
            this._bodies = [];
            this._collisions = [];
            this._activeBodies = [];
            this._constraints = [];
            this._controllers = [];

            this._cachedContacts = [];

            this.setGravity(JNumber3D.getScaleVector(Vector3D.Y_AXIS, -10));

        }

        setCollisionSystem(collisionSystemGrid = false, sx, sy, sz, nx = 20, ny = 20, nz = 20, dx = 200, dy = 200, dz = 200) {
            if (collisionSystemGrid == null) collisionSystemGrid = false;
            if (nx == null) nx = 20;
            if (ny == null) ny = 20;
            if (nz == null) nz = 20;
            if (dx == null) dx = 200;
            if (dy == null) dy = 200;
            if (dz == null) dz = 200;

            // which collisionsystem to use grid / brute
            if (collisionSystemGrid) {
                this._collisionSystem = new CollisionSystemGrid(sx, sy, sz, nx, ny, nz, dx, dy, dz);
            }
            else {
                this._collisionSystem = new CollisionSystemBrute(); // brute by default	
            }

        }

        getCollisionSystem() {

            return this._collisionSystem;

        }

        setGravity(gravity) {

            this._gravity = gravity;
            if (this._gravity.x == this._gravity.y && this._gravity.y == this._gravity.z)
                this._gravityAxis = -1;

            this._gravityAxis = 0;
            if (Math.abs(this._gravity.y) > Math.abs(this._gravity.z))
                this._gravityAxis = 1;

            if (Math.abs(this._gravity.z) > Math.abs(JNumber3D.toArray(this._gravity)[this._gravityAxis]))
                this._gravityAxis = 2;

            // do update only when dirty, faster than call every time in step
            for (var _bodies_i = 0, _bodies_l = this._bodies.length, body; (_bodies_i < _bodies_l) && (body = this._bodies[_bodies_i]); _bodies_i++)
                body.updateGravity(this._gravity, this._gravityAxis);

        }

        get_gravity() {

            return this._gravity;

        }

        get_gravityAxis() {

            return this._gravityAxis;

        }

        get_bodies() {

            return this._bodies;

        }

        get_activeBodies() {

            return this._activeBodies;

        }

        get_constraints() {

            return this._constraints;

        }

        addBody(body: RigidBody) {

            if (this._bodies.indexOf(body) < 0) {
                this._bodies.push(body);
                this._collisionSystem.addCollisionBody(body);

                // update only once, and callback later when dirty
                body.updateGravity(this._gravity, this._gravityAxis);
            }

        }

        removeBody(body) {

            if (this._bodies.indexOf(body) >= 0) {
                this._bodies.splice(this._bodies.indexOf(body), 1);
                this._collisionSystem.removeCollisionBody(body);
            }

        }

        removeAllBodies() {

            this._bodies.length = 0;
            this._collisionSystem.removeAllCollisionBodies();

        }

        addConstraint(constraint) {

            if (this._constraints.indexOf(constraint) < 0)
                this._constraints.push(constraint);

        }

        removeConstraint(constraint) {

            if (this._constraints.indexOf(constraint) >= 0)
                this._constraints.splice(this._constraints.indexOf(constraint), 1);

        }

        removeAllConstraints() {

            for (var _constraints_i = 0, _constraints_l = this._constraints.length, constraint; (_constraints_i < _constraints_l) && (constraint = this._constraints[_constraints_i]); _constraints_i++) {
                constraint.disableConstraint();
            }
            this._constraints.length = 0;

        }

        addController(controller) {

            if (this._controllers.indexOf(controller) < 0)
                this._controllers.push(controller);

        }

        removeController(controller) {

            if (this._controllers.indexOf(controller) >= 0)
                this._controllers.splice(this._controllers.indexOf(controller), 1);

        }

        removeAllControllers() {

            for (var _controllers_i = 0, _controllers_l = this._controllers.length, controller; (_controllers_i < _controllers_l) && (controller = this._controllers[_controllers_i]); _controllers_i++) {
                controller.disableController();
            }
            this._controllers.length = 0;

        }

        setSolverType(type) {

            switch (type) {
                case "FAST":
                    this.preProcessCollisionFn = this.preProcessCollisionFast;
                    this.preProcessContactFn = this.preProcessCollisionFast;
                    this.processCollisionFn = this.processCollisionNormal;
                    this.processContactFn = this.processCollisionNormal;
                    return;
                case "NORMAL":
                    this.preProcessCollisionFn = this.preProcessCollisionNormal;
                    this.preProcessContactFn = this.preProcessCollisionNormal;
                    this.processCollisionFn = this.processCollisionNormal;
                    this.processContactFn = this.processCollisionNormal;
                    return;
                case "ACCUMULATED":
                    this.preProcessCollisionFn = this.preProcessCollisionNormal;
                    this.preProcessContactFn = this.preProcessCollisionAccumulated;
                    this.processCollisionFn = this.processCollisionNormal;
                    this.processContactFn = this.processCollisionAccumulated;
                    return;
                default:
                    this.preProcessCollisionFn = this.preProcessCollisionNormal;
                    this.preProcessContactFn = this.preProcessCollisionNormal;
                    this.processCollisionFn = this.processCollisionNormal;
                    this.processContactFn = this.processCollisionNormal;
                    return;
            }

        }

        moreCollPtPenetration(info0, info1) {

            if (info0.initialPenetration < info1.initialPenetration)
                return 1;
            else if (info0.initialPenetration > info1.initialPenetration)
                return -1;
            else
                return 0;

        }

        preProcessCollisionFast(collision, dt) {

            collision.satisfied = false;

            var body0, body1;

            body0 = collision.objInfo.body0;
            body1 = collision.objInfo.body1;

            var N = collision.dirToBody, tempV;
            var timescale = JConfig.numPenetrationRelaxationTimesteps * dt, approachScale = 0, tiny = JMath3D.NUM_TINY, allowedPenetration = JConfig.allowedPenetration;
            var ptInfo;
            var collision_pointInfo = collision.pointInfo;

            if (collision_pointInfo.length > 3) {
                collision_pointInfo = collision_pointInfo.sort(this.moreCollPtPenetration);
                collision_pointInfo.fixed = false;
                collision_pointInfo.length = 3;
                collision_pointInfo.fixed = true;
            }

            for (let ptInfo of collision_pointInfo) {
                if (!body0.get_movable()) {
                    ptInfo.denominator = 0;
                }
                else {
                    tempV = ptInfo.r0.crossProduct(N);
                    tempV = body0.get_worldInvInertia().transformVector(tempV);
                    ptInfo.denominator = body0.get_invMass() + N.dotProduct(tempV.crossProduct(ptInfo.r0));
                }

                if (body1 && body1.get_movable()) {
                    tempV = ptInfo.r1.crossProduct(N);
                    tempV = body1.get_worldInvInertia().transformVector(tempV);
                    ptInfo.denominator += (body1.get_invMass() + N.dotProduct(tempV.crossProduct(ptInfo.r1)));
                }

                if (ptInfo.denominator < tiny)
                    ptInfo.denominator = tiny;

                if (ptInfo.initialPenetration > allowedPenetration) {
                    ptInfo.minSeparationVel = (ptInfo.initialPenetration - allowedPenetration) / timescale;
                }
                else {
                    approachScale = -0.1 * (ptInfo.initialPenetration - allowedPenetration) / allowedPenetration;

                    if (approachScale < tiny) {
                        approachScale = tiny;
                    }
                    else if (approachScale > 1) {
                        approachScale = 1;
                    }

                    ptInfo.minSeparationVel = approachScale * (ptInfo.initialPenetration - allowedPenetration) / dt;
                }

                if (ptInfo.minSeparationVel > this._maxVelMag)
                    ptInfo.minSeparationVel = this._maxVelMag;
            }

        }

        preProcessCollisionNormal(collision, dt) {

            collision.satisfied = false;

            let body0 = collision.objInfo.body0;
            let body1 = collision.objInfo.body1;

            let N = collision.dirToBody, tempV;
            let timescale = JConfig.numPenetrationRelaxationTimesteps * dt;
            let approachScale = 0;
            let tiny = JMath3D.NUM_TINY;
            let allowedPenetration = JConfig.allowedPenetration;
            let collision_pointInfo = collision.pointInfo;

            for (let ptInfo of collision_pointInfo) {
                if (!body0.get_movable()) {
                    ptInfo.denominator = 0;
                }
                else {
                    tempV = ptInfo.r0.crossProduct(N);
                    tempV = body0.get_worldInvInertia().transformVector(tempV);
                    ptInfo.denominator = body0.get_invMass() + N.dotProduct(tempV.crossProduct(ptInfo.r0));
                }

                if (body1 && body1.get_movable()) {
                    tempV = ptInfo.r1.crossProduct(N);
                    tempV = body1.get_worldInvInertia().transformVector(tempV);
                    ptInfo.denominator += (body1.get_invMass() + N.dotProduct(tempV.crossProduct(ptInfo.r1)));
                }

                if (ptInfo.denominator < tiny)
                    ptInfo.denominator = tiny;

                if (ptInfo.initialPenetration > allowedPenetration) {
                    ptInfo.minSeparationVel = (ptInfo.initialPenetration - allowedPenetration) / timescale;
                }
                else {
                    approachScale = -0.1 * (ptInfo.initialPenetration - allowedPenetration) / allowedPenetration;

                    if (approachScale < tiny) {
                        approachScale = tiny;
                    }
                    else if (approachScale > 1) {
                        approachScale = 1;
                    }
                    ptInfo.minSeparationVel = approachScale * (ptInfo.initialPenetration - allowedPenetration) / dt;
                }

                if (ptInfo.minSeparationVel > this._maxVelMag)
                    ptInfo.minSeparationVel = this._maxVelMag;
            }

        }

        preProcessCollisionAccumulated(collision, dt) {

            collision.satisfied = false;

            let body0 = collision.objInfo.body0;
            let body1 = collision.objInfo.body1;

            var N = collision.dirToBody;
            let timescale = JConfig.numPenetrationRelaxationTimesteps * dt;
            let approachScale = 0;
            let numTiny = JMath3D.NUM_TINY;
            let allowedPenetration = JConfig.allowedPenetration;

            let collision_pointInfo = collision.pointInfo;

            for (let ptInfo of collision_pointInfo) {
                if (!body0.get_movable()) {
                    ptInfo.denominator = 0;
                }
                else {
                    let tempV = ptInfo.r0.crossProduct(N);
                    tempV = body0.get_worldInvInertia().transformVector(tempV);
                    ptInfo.denominator = body0.get_invMass() + N.dotProduct(tempV.crossProduct(ptInfo.r0));
                }

                if (body1 && body1.get_movable()) {
                    let tempV = ptInfo.r1.crossProduct(N);
                    tempV = body1.get_worldInvInertia().transformVector(tempV);
                    ptInfo.denominator += (body1.get_invMass() + N.dotProduct(tempV.crossProduct(ptInfo.r1)));
                }

                if (ptInfo.denominator < numTiny) {
                    ptInfo.denominator = numTiny;
                }

                if (ptInfo.initialPenetration > allowedPenetration) {
                    ptInfo.minSeparationVel = (ptInfo.initialPenetration - allowedPenetration) / timescale;
                }
                else {
                    approachScale = -0.1 * (ptInfo.initialPenetration - allowedPenetration) / allowedPenetration;

                    if (approachScale < numTiny) {
                        approachScale = numTiny;
                    }
                    else if (approachScale > 1) {
                        approachScale = 1;
                    }

                    ptInfo.minSeparationVel = approachScale * (ptInfo.initialPenetration - allowedPenetration) / Math.max(dt, numTiny);
                }

                ptInfo.accumulatedNormalImpulse = 0;
                ptInfo.accumulatedNormalImpulseAux = 0;
                ptInfo.accumulatedFrictionImpulse = new Vector3D();

                var bestDistSq = 0.04;
                var bp = new BodyPair(body0, body1, new Vector3D(), new Vector3D());

                for (let cachedContact of this._cachedContacts) {
                    if (!(bp.body0 == cachedContact.pair.body0 && bp.body1 == cachedContact.pair.body1))
                        continue;

                    var distSq = (cachedContact.pair.body0 == body0) ? cachedContact.pair.r.subtract(ptInfo.r0).get_lengthSquared() : cachedContact.pair.r.subtract(ptInfo.r1).get_lengthSquared();

                    if (distSq < bestDistSq) {
                        bestDistSq = distSq;
                        ptInfo.accumulatedNormalImpulse = cachedContact.impulse.normalImpulse;
                        ptInfo.accumulatedNormalImpulseAux = cachedContact.impulse.normalImpulseAux;
                        ptInfo.accumulatedFrictionImpulse = cachedContact.impulse.frictionImpulse;

                        if (cachedContact.pair.body0 != body0)
                            ptInfo.accumulatedFrictionImpulse = JNumber3D.getScaleVector(ptInfo.accumulatedFrictionImpulse, -1);
                    }
                }

                if (ptInfo.accumulatedNormalImpulse != 0) {
                    var impulse = JNumber3D.getScaleVector(N, ptInfo.accumulatedNormalImpulse);
                    impulse = impulse.add(ptInfo.accumulatedFrictionImpulse);
                    body0.applyBodyWorldImpulse(impulse, ptInfo.r0, false);
                    if (body1)
                        body1.applyBodyWorldImpulse(JNumber3D.getScaleVector(impulse, -1), ptInfo.r1, false);
                }

                if (ptInfo.accumulatedNormalImpulseAux != 0) {
                    impulse = JNumber3D.getScaleVector(N, ptInfo.accumulatedNormalImpulseAux);
                    body0.applyBodyWorldImpulseAux(impulse, ptInfo.r0, false);
                    if (body1)
                        body1.applyBodyWorldImpulseAux(JNumber3D.getScaleVector(impulse, -1), ptInfo.r1, false);
                }
            }

        }

        processCollisionNormal(collision, dt) {

            collision.satisfied = true;

            let body0 = collision.objInfo.body0;
            let body1 = collision.objInfo.body1;

            var gotOne = false;
            var deltaVel = 0, normalVel = 0, finalNormalVel = 0, normalImpulse = 0, tangent_speed, denominator, impulseToReverse, impulseFromNormalImpulse, frictionImpulse, tiny = JMath3D.NUM_TINY;
            var N = collision.dirToBody, impulse, Vr0, Vr1, tempV, VR, tangent_vel, T;

            var collision_pointInfo = collision.pointInfo;
            for (let ptInfo of collision_pointInfo) {
                Vr0 = body0.getVelocity(ptInfo.r0);
                if (body1) {
                    Vr1 = body1.getVelocity(ptInfo.r1);
                    normalVel = Vr0.subtract(Vr1).dotProduct(N);
                } else {
                    normalVel = Vr0.dotProduct(N);
                }
                if (normalVel > ptInfo.minSeparationVel)
                    continue;

                finalNormalVel = -1 * collision.mat.restitution * normalVel;

                if (finalNormalVel < this._minVelForProcessing)
                    finalNormalVel = ptInfo.minSeparationVel;

                deltaVel = finalNormalVel - normalVel;

                if (deltaVel <= this._minVelForProcessing)
                    continue;

                normalImpulse = deltaVel / ptInfo.denominator;

                gotOne = true;
                impulse = JNumber3D.getScaleVector(N, normalImpulse);

                body0.applyBodyWorldImpulse(impulse, ptInfo.r0, false);
                if (body1) body1.applyBodyWorldImpulse(JNumber3D.getScaleVector(impulse, -1), ptInfo.r1, false);

                VR = Vr0.clone();
                if (body1) VR = VR.subtract(Vr1);
                tangent_vel = VR.subtract(JNumber3D.getScaleVector(N, VR.dotProduct(N)));
                tangent_speed = tangent_vel.get_length();

                if (tangent_speed > this._minVelForProcessing) {
                    T = JNumber3D.getDivideVector(tangent_vel, -tangent_speed);
                    denominator = 0;

                    if (body0.get_movable()) {
                        tempV = ptInfo.r0.crossProduct(T);
                        tempV = body0.get_worldInvInertia().transformVector(tempV);
                        denominator = body0.get_invMass() + T.dotProduct(tempV.crossProduct(ptInfo.r0));
                    }

                    if (body1 && body1.get_movable()) {
                        tempV = ptInfo.r1.crossProduct(T);
                        tempV = body1.get_worldInvInertia().transformVector(tempV);
                        denominator += (body1.get_invMass() + T.dotProduct(tempV.crossProduct(ptInfo.r1)));
                    }

                    if (denominator > tiny) {
                        impulseToReverse = tangent_speed / denominator;

                        impulseFromNormalImpulse = collision.mat.friction * normalImpulse;
                        if (impulseToReverse < impulseFromNormalImpulse) {
                            frictionImpulse = impulseToReverse;
                        } else {
                            frictionImpulse = collision.mat.friction * normalImpulse;
                        }
                        T.scaleBy(frictionImpulse);
                        body0.applyBodyWorldImpulse(T, ptInfo.r0, false);
                        if (body1) body1.applyBodyWorldImpulse(JNumber3D.getScaleVector(T, -1), ptInfo.r1, false);
                    }
                }
            }

            if (gotOne) {
                body0.setConstraintsAndCollisionsUnsatisfied();
                if (body1) body1.setConstraintsAndCollisionsUnsatisfied();
            }

            return gotOne;

        }

        processCollisionAccumulated(collision, dt) {

            collision.satisfied = true;

            let body0 = collision.objInfo.body0;
            let body1 = collision.objInfo.body1;

            var gotOne = false;
            var normalVel = 0, finalNormalVel = 0, normalImpulse = 0, tangent_speed, denominator, impulseToReverse, AFIMag, maxAllowedAFIMag, tiny = JMath3D.NUM_TINY;
            var N = collision.dirToBody, impulse, tempV, tangent_vel, frictionImpulseVec, origAccumulatedFrictionImpulse, actualFrictionImpulse;

            var collision_pointInfo = collision.pointInfo;

            for (let ptInfo of collision_pointInfo) {
                let Vr0 = body0.getVelocity(ptInfo.r0);
                if (body1) {
                    let Vr1 = body1.getVelocity(ptInfo.r1);
                    normalVel = Vr0.subtract(Vr1).dotProduct(N);
                } else {
                    normalVel = Vr0.dotProduct(N);
                }
                let deltaVel = -normalVel;

                if (ptInfo.minSeparationVel < 0)
                    deltaVel += ptInfo.minSeparationVel;

                if (Math.abs(deltaVel) > this._minVelForProcessing) {
                    normalImpulse = deltaVel / ptInfo.denominator;
                    var origAccumulatedNormalImpulse = ptInfo.accumulatedNormalImpulse;
                    ptInfo.accumulatedNormalImpulse = Math.max(ptInfo.accumulatedNormalImpulse + normalImpulse, 0);
                    var actualImpulse = ptInfo.accumulatedNormalImpulse - origAccumulatedNormalImpulse;

                    impulse = JNumber3D.getScaleVector(N, actualImpulse);
                    body0.applyBodyWorldImpulse(impulse, ptInfo.r0, false);
                    if (body1) body1.applyBodyWorldImpulse(JNumber3D.getScaleVector(impulse, -1), ptInfo.r1, false);

                    gotOne = true;
                }

                Vr0 = body0.getVelocityAux(ptInfo.r0);
                if (body1) {
                    let Vr1 = body1.getVelocityAux(ptInfo.r1);
                    normalVel = Vr0.subtract(Vr1).dotProduct(N);
                } else {
                    normalVel = Vr0.dotProduct(N);
                }

                deltaVel = -normalVel;

                if (ptInfo.minSeparationVel > 0)
                    deltaVel += ptInfo.minSeparationVel;

                if (Math.abs(deltaVel) > this._minVelForProcessing) {
                    normalImpulse = deltaVel / ptInfo.denominator;
                    origAccumulatedNormalImpulse = ptInfo.accumulatedNormalImpulseAux;
                    ptInfo.accumulatedNormalImpulseAux = Math.max(ptInfo.accumulatedNormalImpulseAux + normalImpulse, 0);
                    actualImpulse = ptInfo.accumulatedNormalImpulseAux - origAccumulatedNormalImpulse;

                    impulse = JNumber3D.getScaleVector(N, actualImpulse);
                    body0.applyBodyWorldImpulseAux(impulse, ptInfo.r0, false);
                    if (body1) body1.applyBodyWorldImpulseAux(JNumber3D.getScaleVector(impulse, -1), ptInfo.r1, false);

                    gotOne = true;
                }

                if (ptInfo.accumulatedNormalImpulse > 0) {
                    Vr0 = body0.getVelocity(ptInfo.r0);
                    let VR = Vr0.clone();
                    if (body1) {
                        let Vr1 = body1.getVelocity(ptInfo.r1);
                        VR = VR.subtract(Vr1);
                    }
                    tangent_vel = VR.subtract(JNumber3D.getScaleVector(N, VR.dotProduct(N)));
                    tangent_speed = tangent_vel.get_length();

                    if (tangent_speed > this._minVelForProcessing) {

                        let T = JNumber3D.getScaleVector(JNumber3D.getDivideVector(tangent_vel, tangent_speed), -1);
                        denominator = 0;
                        if (body0.get_movable()) {
                            tempV = ptInfo.r0.crossProduct(T);
                            tempV = body0.get_worldInvInertia().transformVector(tempV);
                            denominator = body0.get_invMass() + T.dotProduct(tempV.crossProduct(ptInfo.r0));
                        }

                        if (body1 && body1.get_movable()) {
                            tempV = ptInfo.r1.crossProduct(T);
                            tempV = body1.get_worldInvInertia().transformVector(tempV);
                            denominator += (body1.get_invMass() + T.dotProduct(tempV.crossProduct(ptInfo.r1)));
                        }

                        if (denominator > tiny) {
                            impulseToReverse = tangent_speed / denominator;
                            frictionImpulseVec = JNumber3D.getScaleVector(T, impulseToReverse);

                            origAccumulatedFrictionImpulse = ptInfo.accumulatedFrictionImpulse.clone();
                            ptInfo.accumulatedFrictionImpulse = ptInfo.accumulatedFrictionImpulse.add(frictionImpulseVec);

                            AFIMag = ptInfo.accumulatedFrictionImpulse.get_length();
                            maxAllowedAFIMag = collision.mat.friction * ptInfo.accumulatedNormalImpulse;

                            if (AFIMag > tiny && AFIMag > maxAllowedAFIMag)
                                ptInfo.accumulatedFrictionImpulse = JNumber3D.getScaleVector(ptInfo.accumulatedFrictionImpulse, maxAllowedAFIMag / AFIMag);

                            actualFrictionImpulse = ptInfo.accumulatedFrictionImpulse.subtract(origAccumulatedFrictionImpulse);

                            body0.applyBodyWorldImpulse(actualFrictionImpulse, ptInfo.r0, false);
                            if (body1)
                                body1.applyBodyWorldImpulse(JNumber3D.getScaleVector(actualFrictionImpulse, -1), ptInfo.r1, false);
                        }
                    }
                }
            }

            if (gotOne) {
                body0.setConstraintsAndCollisionsUnsatisfied();
                if (body1)
                    body1.setConstraintsAndCollisionsUnsatisfied();
            }

            return gotOne;

        }

        processCollisionForShock(collision, dt) {


            collision.satisfied = true;
            var N = collision.dirToBody;

            var timescale = JConfig.numPenetrationRelaxationTimesteps * dt;
            var body0 = collision.objInfo.body0;
            var body1 = collision.objInfo.body1;

            if (!body0.get_movable())
                body0 = null;
            if (body1 && !body1.get_movable())
                body1 = null;

            if (!body0 && !body1) {
                return false;
            }

            for (let ptInfo of collision.pointInfo) {
                let normalVel = 0;
                if (body0) {
                    normalVel = body0.getVelocity(ptInfo.r0).dotProduct(N) + body0.getVelocityAux(ptInfo.r0).dotProduct(N);
                }
                if (body1) {
                    normalVel -= (body1.getVelocity(ptInfo.r1).dotProduct(N) + body1.getVelocityAux(ptInfo.r1).dotProduct(N));
                }

                let finalNormalVel = (ptInfo.initialPenetration - JConfig.allowedPenetration) / timescale;
                if (finalNormalVel < 0) {
                    continue;
                }
                let impulse = (finalNormalVel - normalVel) / ptInfo.denominator;
                let orig = ptInfo.accumulatedNormalImpulseAux;
                ptInfo.accumulatedNormalImpulseAux = Math.max(ptInfo.accumulatedNormalImpulseAux + impulse, 0);
                let actualImpulse = JNumber3D.getScaleVector(N, ptInfo.accumulatedNormalImpulseAux - orig);

                if (body0)
                    body0.applyBodyWorldImpulse(actualImpulse, ptInfo.r0, false);

                if (body1)
                    body1.applyBodyWorldImpulse(JNumber3D.getScaleVector(actualImpulse, -1), ptInfo.r1, false);
            }

            if (body0)
                body0.setConstraintsAndCollisionsUnsatisfied();
            if (body1)
                body1.setConstraintsAndCollisionsUnsatisfied();
            return true;

        }

        sortPositionX(body0, body1) {

            if (body0.get_currentState().position.x < body1.get_currentState().position.x)
                return -1;
            else if (body0.get_currentState().position.x > body1.get_currentState().position.x)
                return 1;
            else
                return 0;

        }

        sortPositionY(body0, body1) {

            if (body0.get_currentState().position.y < body1.get_currentState().position.y)
                return -1;
            else if (body0.get_currentState().position.y > body1.get_currentState().position.y)
                return 1;
            else
                return 0;

        }

        sortPositionZ(body0, body1) {

            if (body0.get_currentState().position.z < body1.get_currentState().position.z)
                return -1;
            else if (body0.get_currentState().position.z > body1.get_currentState().position.z)
                return 1;
            else
                return 0;

        }

        doShockStep(dt) {

            if (Math.abs(this._gravity.x) > Math.abs(this._gravity.y) && Math.abs(this._gravity.x) > Math.abs(this._gravity.z)) {
                this._bodies = this._bodies.sort(this.sortPositionX);
                this._collisionSystem.collBody = this._collisionSystem.collBody.sort(this.sortPositionX);
            }
            else if (Math.abs(this._gravity.y) > Math.abs(this._gravity.z) && Math.abs(this._gravity.y) > Math.abs(this._gravity.x)) {
                this._bodies = this._bodies.sort(this.sortPositionY);
                this._collisionSystem.collBody = this._collisionSystem.collBody.sort(this.sortPositionY);
            }
            else if (Math.abs(this._gravity.z) > Math.abs(this._gravity.x) && Math.abs(this._gravity.z) > Math.abs(this._gravity.y)) {
                this._bodies = this._bodies.sort(this.sortPositionZ);
                this._collisionSystem.collBody = this._collisionSystem.collBody.sort(this.sortPositionZ);
            }

            var gotOne = true;

            for (let body of this._bodies) {
                if (body.get_movable()) {
                    if (body.collisions.length == 0 || !body.isActive) {
                        body.internalSetImmovable();
                    }
                    else {
                        let setImmovable = false;
                        for (let info of body.collisions) {
                            let body0 = info.objInfo.body0;
                            let body1 = info.objInfo.body1;

                            if ((body0 == body && (!body1 || !body1.get_movable())) || (body1 == body && (!body0 || !body0.get_movable()))) {
                                this.preProcessCollisionFn(info, dt);
                                this.processCollisionForShock(info, dt);
                                setImmovable = true;
                            }
                        }

                        if (setImmovable) {
                            body.internalSetImmovable();
                        }
                    }
                }
            }

            for (let body of this._bodies) {
                body.internalRestoreImmovable();
            }

        }

        updateContactCache() {

            this._cachedContacts = [];

            var contact;
            var i = 0;
            for (let collInfo of this._collisions) {
                let collInfo_objInfo = collInfo.objInfo;
                let body0 = collInfo_objInfo.body0;
                let body1 = collInfo_objInfo.body1;

                let collInfo_pointInfo = collInfo.pointInfo;
                //this._cachedContacts.fixed = false;
                this._cachedContacts.length += collInfo_pointInfo.length;
                //this._cachedContacts.fixed = true;

                for (let ptInfo of collInfo_pointInfo) {
                    let id1 = -1;
                    if (body1)
                        id1 = body1.get_id();

                    let fricImpulse = (body0.get_id() > id1) ? ptInfo.accumulatedFrictionImpulse : JNumber3D.getScaleVector(ptInfo.accumulatedFrictionImpulse, -1);

                    this._cachedContacts[i++] = contact = new ContactData();
                    contact.pair = new BodyPair(body0, body1, ptInfo.r0, ptInfo.r1);
                    contact.impulse = new CachedImpulse(ptInfo.accumulatedNormalImpulse, ptInfo.accumulatedNormalImpulseAux, ptInfo.accumulatedFrictionImpulse);
                }
            }

        }

        handleAllConstraints(dt: number, iter: number, forceInelastic: boolean) {

            var origNumCollisions = this._collisions.length;
            let iteration = JConfig.numConstraintIterations;
            var collInfo;
            var flag;

            if (this._constraints.length > 0) {
                for (let constraint of this._constraints)
                    constraint.preApply(dt);

                for (let step = 0; step < iteration; step++) {
                    let gotOne = false;
                    for (let constraint of this._constraints) {
                        if (!constraint.satisfied) {
                            flag = constraint.apply(dt);
                            gotOne = gotOne || flag;
                        }
                    }
                    if (!gotOne)
                        break;
                }
            }

            if (forceInelastic) {
                for (let collInfo of this._collisions) {
                    this.preProcessContactFn(collInfo, dt);
                    collInfo.mat.restitution = 0;
                    collInfo.satisfied = false;
                }
            }
            else {
                for (let collInfo of this._collisions)
                    this.preProcessCollisionFn(collInfo, dt);
            }

            for (let step = 0; step < iter; step++) {
                let gotOne = true;

                for (let collInfo of this._collisions) {
                    if (!collInfo.satisfied) {
                        if (forceInelastic)
                            flag = this.processContactFn(collInfo, dt);
                        else
                            flag = this.processCollisionFn(collInfo, dt);

                        gotOne = gotOne || flag;
                    }
                }

                let len = this._collisions.length;
                if (forceInelastic) {
                    for (let i = origNumCollisions; i < len; i++) {
                        collInfo = this._collisions[i];
                        collInfo.mat.restitution = 0;
                        collInfo.satisfied = false;
                        this.preProcessContactFn(collInfo, dt);
                    }
                }
                else {
                    for (let i = origNumCollisions; i < len; i++)
                        this.preProcessCollisionFn(this._collisions[i], dt);
                }

                origNumCollisions = len;

                if (!gotOne)
                    break;
            }

        }

        activateObject(body: RigidBody) {

            if (!body.get_movable() || body.isActive)
                return;

            if (this._activeBodies.indexOf(body) < 0) {
                body.setActive();
                //this._activeBodies.fixed = false;
                this._activeBodies.push(body);
                //this._activeBodies.fixed = true;
            }

        }

        tryToActivateAllFrozenObjects() {

            for (let body of this._bodies) {
                if (!body.isActive) {
                    if (body.getShouldBeActive()) {
                        this.activateObject(body);
                    }
                    else {
                        body.setLineVelocity(new Vector3D());
                        body.setAngleVelocity(new Vector3D());
                    }
                }
            }

        }

        tryToFreezeAllObjects(dt) {
            for (let activeBody of this._activeBodies) {
                activeBody.dampForDeactivation();
                activeBody.tryToFreeze(dt);
            }

        }

        activateAllFrozenObjectsLeftHanging() {

            for (let body of this._activeBodies) {
                body.doMovementActivations(this);
                let body_collisions = body.collisions;
                if (body_collisions.length > 0) {

                    for (let collisionInfo of body_collisions) {
                        let other_body = collisionInfo.objInfo.body0;
                        if (other_body == body)
                            other_body = collisionInfo.objInfo.body1;

                        if (!other_body.isActive)
                            body.addMovementActivation(body.get_currentState().position, other_body);
                    }
                }
            }

        }

        updateAllController(dt) {
            for (let controller of this._controllers)
                controller.updateController(dt);

        }

        updateAllVelocities(dt) {
            for (let activeBody of this._activeBodies)
                activeBody.updateVelocity(dt);
        }

        notifyAllPostPhysics(dt) {
            for (let activeBody of this._activeBodies)
                activeBody.postPhysics(dt);
        }

        detectAllCollisions(dt) {

            for (let body of this._bodies) {
                if (body.isActive) {
                    body.storeState();
                    body.updateVelocity(dt);
                    body.updatePositionWithAux(dt);
                }
                body.collisions.length = 0;
            }

            this._collisions.length = 0;
            this._collisionSystem.detectAllCollisions(this._activeBodies, this._collisions);

            for (let activeBody of this._activeBodies)
                activeBody.restoreState();

        }

        findAllActiveBodiesAndCopyStates() {

            this._activeBodies = [];
            //var i = 0;

            for (let body of this._bodies) {
                // findAllActiveBodies
                if (body.isActive) {
                    //this._activeBodies[i++] = body;
                    this._activeBodies.push(body);
                    body.copyCurrentStateToOld();
                }

            }

            // correct length
            //this._activeBodies.fixed = false;
            //this._activeBodies.length = i;

            // fixed is faster
            //this._activeBodies.fixed = true;

        }

        integrate(dt) {

            this._doingIntegration = true;

            this.findAllActiveBodiesAndCopyStates();
            this.updateAllController(dt);
            this.detectAllCollisions(dt);
            this.handleAllConstraints(dt, JConfig.numCollisionIterations, false);
            this.updateAllVelocities(dt);
            this.handleAllConstraints(dt, JConfig.numContactIterations, true);

            if (JConfig.doShockStep)
                this.doShockStep(dt);

            this.tryToActivateAllFrozenObjects();
            this.tryToFreezeAllObjects(dt);
            this.activateAllFrozenObjectsLeftHanging();

            this.notifyAllPostPhysics(dt);

            if (JConfig.solverType == "ACCUMULATED")
                this.updateContactCache();

            this._doingIntegration = false;

        }

        static _currentPhysicsSystem: PhysicsSystem = null; // PhysicsSystem

        static getInstance() {

            if (!this._currentPhysicsSystem) {
                //trace("version: JigLibFlash fp11 (2011-7-14)");
                this._currentPhysicsSystem = new PhysicsSystem();
            }
            return this._currentPhysicsSystem;

        }
    }
}