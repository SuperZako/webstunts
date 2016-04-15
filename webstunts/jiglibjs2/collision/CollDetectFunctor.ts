

module jiglib {

    export abstract class CollDetectFunctor {

        constructor(public name: string, public type0: string, public type1: string) {
        }
        abstract collDetect(info: CollDetectInfo, collArr);
    }
}