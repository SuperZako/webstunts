

module jiglib {
    export function extend(dest, source) {
        for (var proto in source.prototype) {
            dest.prototype[proto] = source.prototype[proto];
        }
    };

    export function trace(message) { };

}