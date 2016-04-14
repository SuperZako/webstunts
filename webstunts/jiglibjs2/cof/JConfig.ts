
namespace jiglib {
    export class JConfig {
        static solverType = "ACCUMULATED";
        static rotationType = "DEGREES";
        static doShockStep = false;
        static allowedPenetration = 0.01;
        static collToll = 0.05;
        static velThreshold = 0.5;
        static angVelThreshold = 0.5;
        static posThreshold = 0.2;
        static orientThreshold = 0.2;
        static deactivationTime = 10.0;
        static numPenetrationRelaxationTimesteps = 10;
        static numCollisionIterations = 1;
        static numContactIterations = 2;
        static numConstraintIterations = 2;

    }
}