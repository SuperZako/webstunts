
module jiglib {
    export class JMath3D {


        static NUM_TINY = 0.000001; // Number
        static NUM_HUGE = 1000000; // Number

        static getLimiteNumber(num: number, min: number, max: number) {

            var n = num;
            if (n < min) {
                n = min;
            }
            else if (n > max) {
                n = max;
            }
            return n;

        }

    }
}
