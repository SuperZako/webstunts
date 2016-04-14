
/// <reference path="../physics/RigidBody.ts"/>

module jiglib {

    export class CollisionSystemGridEntry {

        collisionBody: RigidBody = null; // RigidBody
        previous: CollisionSystemGridEntry = null; // CollisionSystemGridEntry
        next: CollisionSystemGridEntry = null; // CollisionSystemGridEntry
        gridIndex: number = null; // int

        constructor(collisionBody) {
            this.collisionBody = collisionBody;
            this.previous = this.next = null;
        }



        static removeGridEntry(entry) {

            // link the CollisionSystemGridEntry.previous to the CollisionSystemGridEntry.next (may be 0)
            entry.previous.next = entry.next;
            // link the CollisionSystemGridEntry.next (if it exists) to the CollisionSystemGridEntry.previous.
            if (entry.next != null)
                entry.next.previous = entry.previous;
            // tidy up this entry
            entry.previous = entry.next = null;
            entry.gridIndex = -2;

        }

        static insertGridEntryAfter(entry, prev) {

            var next = prev.next;
            prev.next = entry;
            entry.previous = prev;
            entry.next = next;
            if (next != null)
                next.previous = entry;
            entry.gridIndex = prev.gridIndex;

        }
    }
}