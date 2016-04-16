
namespace jiglib {

    //var JConstraintWorldPoint = jiglib.JConstraintWorldPoint;
    //var JConstraintMaxDistance = jiglib.JConstraintMaxDistance;
    //var JConstraintPoint = jiglib.JConstraintPoint;

    export class JConstraint {
        satisfied = null; // Boolean
        _constraintEnabled = null; // Boolean
        constructor() {
            //this.satisfied = null; // Boolean
            //this._constraintEnabled = null; // Boolean
        }

        preApply(dt) {

            this.satisfied = false;

        }

        apply(dt) {

            return false;

        }

        enableConstraint() {


        }

        disableConstraint() {


        }

        get_constraintEnabled() {

            return this._constraintEnabled;

        }

    }
}