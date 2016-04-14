
module jiglib {

    export class JCollisionEvent {
        body = null; // RigidBody
        constructor(type) {
        }
        static COLLISION_START = "collisionStart"; // String
        static COLLISION_END = "collisionEnd"; // String
    }
}